## 使用RPi.GPIO package
## 馬達從0°每次遞增+10°直到180°
## mg996r 必須是 180 規格的

import RPi.GPIO as GPIO
import time

servo_pin = 18

GPIO.setmode(GPIO.BCM)
GPIO.setup(servo_pin, GPIO.OUT)

pwm_servo = GPIO.PWM(servo_pin, 50) #50Hz frequency
pwm_servo.start(0)

def destroy():
    pwm_servo.stop()
    GPIO.cleanup()


# input 0~180 degree (DO NOT exceed 180)
def set_angle(angle):
    # duty_cycle description
    # 0 = stop
    # 2 = 0 degree
    # 7 = 90 degree
    # 12 = 180 degree
    duty = 2 + (angle / 18)
    pwm_servo.ChangeDutyCycle(duty)
    # to eliminate vibration
    time.sleep(0.3)
    pwm_servo.ChangeDutyCycle(0)
    print("angle = ", angle, " -> duty = ", duty)

def run():
    for angle in range(0, 181, 10):
        set_angle(angle)
        time.sleep(1)

if __name__ == "__main__":
    try:
        run()
    finally:
        destroy()

