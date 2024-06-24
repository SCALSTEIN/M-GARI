import RPi.GPIO as GPIO
import time

# Define GPIO pins for motor control
left_motor_pins = (12, 16, 20)  # GPIO pins for left motor (IN1, IN2, ENA)
right_motor_pins = (21, 19, 26)  # GPIO pins for right motor (IN3, IN4, ENB)

# Set GPIO mode and initial setup
GPIO.setmode(GPIO.BCM)
GPIO.setup(left_motor_pins + right_motor_pins, GPIO.OUT)
GPIO.setup(17, GPIO.OUT)
GPIO.setup(27, GPIO.IN)

# Initialize PWM for motor speed control
left_pwm = GPIO.PWM(left_motor_pins[2], 100)
right_pwm = GPIO.PWM(right_motor_pins[2], 100)
left_pwm.start(0)
right_pwm.start(0)

# Function to move robot forward
def move_forward():
    GPIO.output(left_motor_pins[0], GPIO.HIGH)
    GPIO.output(left_motor_pins[1], GPIO.LOW)
    GPIO.output(right_motor_pins[0], GPIO.HIGH)
    GPIO.output(right_motor_pins[1], GPIO.LOW)
    left_pwm.ChangeDutyCycle(50)
    right_pwm.ChangeDutyCycle(50)

# Function to stop robot
def stop_robot():
    GPIO.output(left_motor_pins[0], GPIO.LOW)
    GPIO.output(left_motor_pins[1], GPIO.LOW)
    GPIO.output(right_motor_pins[0], GPIO.LOW)
    GPIO.output(right_motor_pins[1], GPIO.LOW)
    left_pwm.ChangeDutyCycle(0)
    right_pwm.ChangeDutyCycle(0)

# Function to check obstacle distance using ultrasonic sensor
def check_obstacle():
    GPIO.output(17, GPIO.HIGH)
    time.sleep(0.00001)
    GPIO.output(17, GPIO.LOW)

    while GPIO.input(27) == 0:
        pulse_start = time.time()

    while GPIO.input(27) == 1:
        pulse_end = time.time()

    pulse_duration = pulse_end - pulse_start
    distance = pulse_duration * 17150  # Speed of sound = 34300 cm/s
    distance = round(distance, 2)
    return distance

# Main loop for autonomous robot control
try:
    while True:
        distance = check_obstacle()
        print(f"Distance: {distance} cm")

        if distance > 15:  # Adjust distance threshold as needed
            move_forward()
        else:
            stop_robot()
            time.sleep(1)  # Pause for obstacle avoidance

except KeyboardInterrupt:
    print("Program terminated by user.")

finally:
    stop_robot()
    GPIO.cleanup()
