import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)

ENA = 18
IN1 = 17
IN2 = 27

ENB = 13
IN3 = 5
IN4 = 6

GPIO.setup([ENA, IN1, IN2, ENB, IN3, IN4], GPIO.OUT)

pwmA = GPIO.PWM(ENA, 1000)
pwmB = GPIO.PWM(ENB, 1000)
pwmA.start(0)
pwmB.start(0)

def motor_a_forward(speed=50):
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    pwmA.ChangeDutyCycle(speed)

def motor_a_backward(speed=50):
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    pwmA.ChangeDutyCycle(speed)

def motor_b_forward(speed=50):
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)
    pwmB.ChangeDutyCycle(speed)

def motor_b_backward(speed=50):
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    pwmB.ChangeDutyCycle(speed)

def stop_motors():
    pwmA.ChangeDutyCycle(0)
    pwmB.ChangeDutyCycle(0)
    GPIO.output([IN1, IN2, IN3, IN4], GPIO.LOW)

try:
    print("Motors forward")
    motor_a_forward(80)
    motor_b_forward(90)
    time.sleep(2)

    print("Motors backward")
    motor_a_backward(80)
    motor_b_backward(90)
    time.sleep(2)

    print("Stop motors")
    stop_motors()

finally:
    pwmA.stop()
    pwmB.stop()
    GPIO.cleanup()
