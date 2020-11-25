import time
import RPi.GPIO as GPIO
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)

class Robot():
    
    # Pin configurations
    # Left motor enable, controll pins and PWM for the motor driver's respective pins
    ENA = 23
    IN1 = 26
    IN2 = 29
    PWMA = 33
    # Left motor enable, controll pins and PWM for the motor driver's respective pins
    ENB = 35
    IN3 = 37
    IN4 = 38
    PWMB = 32
    
    
    def __init__(self, *args, **kwargs):
        super(Robot, self).__init__(*args, **kwargs)
        
        # Setting up the left motor
        GPIO.setup(ENA, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(IN1, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(IN2, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(PWMA, GPIO.OUT)
        self.left_speed = 0 # Variable for controlling the left motor speed
        
        # Setting up the right motor
        GPIO.setup(ENB, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(IN3, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(IN4, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(PWMB, GPIO.OUT)
        self.right_speed = 0 # Variable for controlling the right motor speed
        
        # Setting up the PWM
        self.pwm = [GPIO.PWM(PWMB,50), GPIO.PWM(PWMA,50)]
        self.pwm[0].start(0)
        self.pwm[1].start(1)
        
    def set_motors(self, left_speed=1.0, right_speed=1.0):
        GPIO.output(self.lef
        self.right_motor.value = right_speed
        
    def forward(self, speed=1.0, duration=None):
        self.left_motor.value = speed
        self.right_motor.value = speed

    def backward(self, speed=1.0):
        self.left_motor.value = -speed
        self.right_motor.value = -speed

    def left(self, speed=1.0):
        self.left_motor.value = -speed
        self.right_motor.value = speed

    def right(self, speed=1.0):
        self.left_motor.value = speed
        self.right_motor.value = -speed

    def stop(self):
        self.left_motor.value = 0
        self.right_motor.value = 0
