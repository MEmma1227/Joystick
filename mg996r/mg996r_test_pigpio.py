## 使用 pigpio package
## 馬達從0°每次遞增+10°直到180°
## mg996r 必須是 180 規格的

## 安裝 pigpiod
## sudo apt-get install pigpio python-pigpio python3-pigpio

## 執行此檔案前要先開啟 pigpio 的 Server
## sudo pigpiod
## 結束後要關掉 Server
## sudo killall pigpiod

import pigpio
import time

servo_pin = 18

pwm = pigpio.pi()
pwm.set_mode(servo_pin, pigpio.OUTPUT)
pwm.set_PWM_frequency(servo_pin,50) #50Hz frequency

def destroy():
    pwm.set_PWM_dutycycle(servo_pin, 0)
    pwm.set_PWM_frequency(servo_pin, 0)
    pwm.stop()

# input 0~180 degree (DO NOT exceed 180)
def set_angle(angle):
    # duty_cycle description
    # 0 = stop
    # 500 = 0 degree
    # 1500 = 90 degree
    # 2500 = 180 degree
    duty = 500 + (float(angle) / 180.0) * 2000  # python2 需要用float寫, python3 則可直接除掉
    pwm.set_servo_pulsewidth(servo_pin, duty)
    print("angle = ", angle, " -> duty = ", duty)
    time.sleep(0.5)

def run():
    for angle in range(0, 181, 10):
        set_angle(angle)
        time.sleep(1)

if __name__ == "__main__":
    try:
        run()
    finally:
        destroy()

