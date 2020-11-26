# Author: Balázs Peisz
# Date: 2020.11.26.
# Disclaimer: This is a modified version of the original robot.py script which ships with the jetpack SDK

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
        GPIO.output(IN1, GPIO.HIGH)
        GPIO.output(IN2, GPIO.LOW)
        GPIO.output(IN3, GPIO.LOW)
        GPIO.output(IN4, GPIO.HIGH)
        self.left_speed = ((left_speed - (-1))/2)*100
        self.right_speed = ((right_speed - (-1))/2)*100
        print()
        print()
        self.pwm[0].ChangeDutyCycle(self.left_speed)
        self.pwm[1].ChangeDutyCycle(self.right_speed)
        
    def forward(self, speed=1.0, duration=None):
        # Setting the motor driver pins to dorward
        GPIO.output(IN1, GPIO.HIGH)
        GPIO.output(IN2, GPIO.LOW)
        GPIO.output(IN3, GPIO.LOW)
        GPIO.output(IN4, GPIO.HIGH)
        
        # Setting the speed for the motors
        self.speed = ((speed - (-1))/2)*100
        self.pwm[0].ChangeDutyCycle(self.speed)
        self.pwm[1].ChangeDutyCycle(self.speed)

    def backward(self, speed=1.0):
        # Setting the motor driver pins to backward
        GPIO.output(IN1, GPIO.LOW)
        GPIO.output(IN2, GPIO.HIGH)
        GPIO.output(IN3, GPIO.HIGH)
        GPIO.output(IN4, GPIO.LOW)
        
        # Setting the speed for the motors
        self.speed = ((speed - (-1))/2)*100
        self.pwm[0].ChangeDutyCycle(self.speed)
        self.pwm[1].ChangeDutyCycle(self.speed)

    def left(self, speed=1.0):
        # Setting the right motor to forward and the left one to backward
        GPIO.output(IN1, GPIO.LOW) # Left backward
        GPIO.output(IN2, GPIO.HIGH)
        GPIO.output(IN3, GPIO.LOW) # Right forward
        GPIO.output(IN4, GPIO.HIGH)
        
        # Seting the speed for the motors
        self.speed = ((speed - (-1))/2)*100
        self.pwm[0].ChangeDutyCycle(self.speed)
        self.pwm[1].ChangeDutyCycle(self.speed)

    def right(self, speed=1.0):
        # Setting the right motor to backward and the left one to forward
        GPIO.output(IN1, GPIO.HIGH) # Left forward
        GPIO.output(IN2, GPIO.LOW)
        GPIO.output(IN3, GPIO.HIGH) # Right backward
        GPIO.output(IN4, GPIO.LOW)
        
        # Seting the speed for the motors
        self.speed = ((speed - (-1))/2)*100
        self.pwm[0].ChangeDutyCycle(self.speed)
        self.pwm[1].ChangeDutyCycle(self.speed)

    def stop(self):
        #Stopping both motors
        GPIO.output(IN1, GPIO.LOW)
        GPIO.output(IN2, GPIO.LOW)
        GPIO.output(IN3, GPIO.LOW)
        GPIO.output(IN4, GPIO.LOW)
        
        # Setting the motors speed to zero
        self.left_speed = 0
        self.right_speed = 0
        self.pwm[0].ChangeDutyCycle(self.left_speed)
        self.pwm[1].ChangeDutyCycle(self.right_speed)
