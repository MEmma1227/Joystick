import pigpio
import time

servo_pin = 18

pwm = pigpio.pi()
pwm.set_mode(servo_pin, pigpio.OUTPUT)
pwm.set_PWM_frequency(servo_pin, 50)  # 50Hz frequency

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
    while True:
        user_input = input("請輸入角度(0~180)，或輸入'q'退出: ")

        if user_input.lower() == 'q':  # 檢查是否輸入'q'以退出程式
            print("程式結束")
            break

        try:
            angle = int(user_input)
            if 0 <= angle <= 180:
                set_angle(angle)
            else:
                print("請輸入有效的角度值 (0~180)")
        except ValueError:
            print("無效的輸入，請輸入數字或'q'退出")

if __name__ == "__main__":
    try:
        run()
    finally:
        destroy()
