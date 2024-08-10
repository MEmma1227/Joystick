# -*- coding: utf-8 -*-

import RPi.GPIO as GPIO
from time import sleep, time
from math import pi

counter = 0
last_counter = 0  # 用來儲存前一次的計數器值
Enc_A = 17  
Enc_B = 27  
wheel_radius = 323.0/2000  # 車輪半徑 (直徑為323 mm)
speed_sampling_time = 1.0

last_state = 0b00  # 二進位的形式表示目前狀態

def init():
    print("Rotary Encoder Test Program")
    GPIO.setwarnings(True)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(Enc_A, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(Enc_B, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.add_event_detect(Enc_A, GPIO.BOTH, callback=rotation_decode)
    GPIO.add_event_detect(Enc_B, GPIO.BOTH, callback=rotation_decode)
    return

def rotation_decode(channel):
    global counter, last_state
    
    # 讀取目前的A和B訊號
    a_state = GPIO.input(Enc_A)
    b_state = GPIO.input(Enc_B)
    
    # 將目前的A和B訊號組合成一個二進制數
    current_state = (a_state << 1) | b_state
    
    # 組合上一次的狀態和目前的狀態來判斷方向
    state_transition = (last_state << 2) | current_state
    
    # 順時鐘： 00 -> 10 -> 11 -> 01
    # 逆時鐘： 00 -> 01 -> 11 -> 10

    # 根據狀態機的變化判斷方向
    if state_transition in [0b1101, 0b0100, 0b0010, 0b1011]:
        counter += 1
        print("direction -> ", counter)
    elif state_transition in [0b1110, 0b0111, 0b0001, 0b1000]:
        counter -= 1
        print("direction <- ", counter)
    
    # 更新最後狀態
    last_state = current_state

def calculate_speed():
    global last_counter
    current_counter = counter
    delta_counter = current_counter - last_counter
    last_counter = current_counter

    # 計算角速度 (rad/s)
    angular_velocity = (delta_counter / (speed_sampling_time * 4000.0)) * 2 * pi  # 弧度/秒

    # 計算線速度 (m/s)
    linear_velocity = angular_velocity * wheel_radius  # 米/秒
    
    return angular_velocity, linear_velocity

def main():
    try:
        init()
        while True:
            sleep(speed_sampling_time)  # 每秒計算一次
            
            angular_velocity, linear_velocity = calculate_speed()
            print(f"Angular Velocity: {angular_velocity:.2f} rad/s, Linear Velocity: {linear_velocity:.2f} m/s")

    except KeyboardInterrupt:
        GPIO.cleanup()

if __name__ == '__main__':
    main()
