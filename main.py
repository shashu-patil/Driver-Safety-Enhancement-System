import cv2
import mediapipe as mp
from scipy.spatial import distance
import time
import RPi.GPIO as GPIO

# GPIO Setup
BUZZER_PIN = 18  # Pin for the buzzer
MOTOR_IN1 = 23   # IN1 pin of L293D for motor direction
MOTOR_IN2 = 24   # IN2 pin of L293D for motor direction
ENABLE_PIN = 25  # EN1 pin of L293D for motor speed
LED_PIN = 26     # Pin for the LED
ALCOHOL_SENSOR_PIN = 17  # Pin for the alcohol sensor (digital output)

GPIO.setmode(GPIO.BCM)
GPIO.setup(BUZZER_PIN, GPIO.OUT)
GPIO.setup(MOTOR_IN1, GPIO.OUT)
GPIO.setup(MOTOR_IN2, GPIO.OUT)
GPIO.setup(ENABLE_PIN, GPIO.OUT) 
GPIO.setup(LED_PIN, GPIO.OUT)
GPIO.setup(ALCOHOL_SENSOR_PIN, GPIO.IN)  # Set the alcohol sensor pin as input

# PWM setup for motor speed
motor_pwm = GPIO.PWM(ENABLE_PIN, 100)  # PWM at 100Hz
motor_pwm.start(100)  # Start with full speed (100% duty cycle)


# Function to control motor
def set_motor(speed_percentage, direction=True):
    """
    Sets the motor speed and direction.
    :param speed_percentage: Speed of motor (0 to 100).
    :param direction: True for forward, False for reverse.
    """
    motor_pwm.ChangeDutyCycle(speed_percentage)
    if direction:
        GPIO.output(MOTOR_IN1, GPIO.HIGH)
        GPIO.output(MOTOR_IN2, GPIO.LOW)
    else:
        GPIO.output(MOTOR_IN1, GPIO.LOW)
        GPIO.output(MOTOR_IN2, GPIO.HIGH)

# Function to calculate the eye aspect ratio (EAR)
def eye_aspect_ratio(eye):
    A = distance.euclidean(eye[1], eye[5])
    B = distance.euclidean(eye[2], eye[4])
    C = distance.euclidean(eye[0], eye[3])
    ear = (A + B) / (2.0 * C)
    return ear

# Thresholds for drowsiness detection
EAR_THRESHOLD = 0.25  # EAR threshold for closed eyes
DROWSINESS_DURATION = 3.0  # Time in seconds before alert is triggered

# Alcohol detection threshold
ALCOHOL_DETECTED = GPIO.HIGH  # High signal from the alcohol sensor indicates alcohol presence

# Initialize MediaPipe Face Mesh
mp_face_mesh = mp.solutions.face_mesh
face_mesh = mp_face_mesh.FaceMesh(static_image_mode=False, max_num_faces=1, refine_landmarks=True, min_detection_confidence=0.5)

# Indices for left and right eyes in the MediaPipe Face Mesh
LEFT_EYE = [362, 385, 387, 263, 373, 380]
RIGHT_EYE = [33, 160, 158, 133, 153, 144]

# Initialize counters
start_time = None
alert_triggered = False

# Start video capture
cap = cv2.VideoCapture(0)

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # Convert the frame to RGB as MediaPipe works with RGB images
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = face_mesh.process(rgb_frame)

        # Check alcohol sensor status (HIGH means alcohol detected)
        if GPIO.input(ALCOHOL_SENSOR_PIN) == ALCOHOL_DETECTED:
            GPIO.output(BUZZER_PIN, GPIO.HIGH)  # Turn on the buzzer
            set_motor(0)  # Stop motor
            GPIO.output(LED_PIN, GPIO.HIGH)  # Blink LED
            time.sleep(0.5)
            GPIO.output(LED_PIN, GPIO.LOW)
            time.sleep(0.5)
            cv2.putText(frame, "ALERT: Alcohol Detected!", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        if results.multi_face_landmarks:
            for face_landmarks in results.multi_face_landmarks:
                # Get the coordinates of the left and right eyes
                left_eye = [(int(face_landmarks.landmark[i].x * frame.shape[1]), 
                             int(face_landmarks.landmark[i].y * frame.shape[0])) for i in LEFT_EYE]
                right_eye = [(int(face_landmarks.landmark[i].x * frame.shape[1]), 
                              int(face_landmarks.landmark[i].y * frame.shape[0])) for i in RIGHT_EYE]

                # Calculate the EAR for both eyes
                left_ear = eye_aspect_ratio(left_eye)
                right_ear = eye_aspect_ratio(right_eye)
                ear = (left_ear + right_ear) / 2.0

                # Check if the EAR is below the threshold, indicating closed eyes
                if ear < EAR_THRESHOLD:
                    if start_time is None:
                        start_time = time.time()  # Start timer if eyes are closed
                    elapsed_time = time.time() - start_time

                    # If eyes are closed for more than the threshold duration, trigger alert
                    if elapsed_time >= DROWSINESS_DURATION:
                        alert_triggered = True
                        GPIO.output(BUZZER_PIN, GPIO.HIGH)  # Turn on the buzzer
                        set_motor(50)  # Reduce motor speed to 50%
                        
                        # Blink LED with 1-second delay
                        GPIO.output(LED_PIN, GPIO.HIGH)
                        time.sleep(0.5)
                        GPIO.output(LED_PIN, GPIO.LOW)
                        time.sleep(0.5)

                        cv2.putText(frame, "ALERT: Wake Up!", (10, 30),
                                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                else:
                    start_time = None  # Reset timer if eyes are open
                    alert_triggered = False
                    GPIO.output(BUZZER_PIN, GPIO.LOW)  # Turn off the buzzer
                    GPIO.output(LED_PIN, GPIO.LOW)  # Turn off the LED
                    set_motor(100)  # Resume motor speed to 100%

                # Display EAR on the frame
                cv2.putText(frame, f"EAR: {ear:.2f}", (500, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        # Display frame
        cv2.imshow("Drowsiness and Alcohol Detection", frame)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
finally:
    # Release resources
    cap.release()
    cv2.destroyAllWindows()
    motor_pwm.stop()
    GPIO.cleanup()
