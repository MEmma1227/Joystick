# -*- coding: utf-8 -*-
# Test for Two-Wheel Differential Drive Mobile Robot 

import RPi.GPIO as GPIO
from time import sleep, time
from math import pi

# 輪子的參數
wheel_radius = 323.0/2000  # 車輪半徑 (直徑為323 mm)
wheel_base = 540.0/1000  # 兩輪之間的距離（540 mm）
speed_sampling_time = 1.0  # 採樣時間間隔（秒）

# 左輪編碼器接腳
Enc_A = 17  
Enc_B = 27  
counter_left = 0
last_counter_left = 0

# 右輪編碼器接腳
Enc_C = 22  
Enc_D = 23  
counter_right = 0
last_counter_right = 0

last_state_left = 0b00  # 左輪編碼器初始狀
last_state_right = 0b00  # 右輪編碼器初始狀態


def init():
    print("Rotary Encoder Test Program")
    GPIO.setwarnings(True)
    GPIO.setmode(GPIO.BCM)
    
    # 初始化左輪編碼器引腳
    GPIO.setup(Enc_A, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(Enc_B, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.add_event_detect(Enc_A, GPIO.BOTH, callback=rotation_decode_left)
    GPIO.add_event_detect(Enc_B, GPIO.BOTH, callback=rotation_decode_left)

    # 初始化右輪編碼器引腳
    GPIO.setup(Enc_C, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(Enc_D, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.add_event_detect(Enc_C, GPIO.BOTH, callback=rotation_decode_right)
    GPIO.add_event_detect(Enc_D, GPIO.BOTH, callback=rotation_decode_right)

    return


def rotation_decode_left(channel):
    global counter_left, last_state_left
    
    # 讀取左輪目前的A和B訊號
    a_state = GPIO.input(Enc_A)
    b_state = GPIO.input(Enc_B)
    
    # 將目前的A和B訊號組合成一個二進制數
    current_state = (a_state << 1) | b_state
    
    # 組合上一次的狀態和目前的狀態來判斷方向
    state_transition = (last_state_left << 2) | current_state
    
    # 根據狀態機的變化判斷方向
    if state_transition in [0b1101, 0b0100, 0b0010, 0b1011]:
        counter_left += 1
    elif state_transition in [0b1110, 0b0111, 0b0001, 0b1000]:
        counter_left -= 1
    
    # 更新最後狀態
    last_state_left = current_state


def rotation_decode_right(channel):
    global counter_right, last_state_right
    
    # 讀取右輪目前的A和B訊號
    a_state = GPIO.input(Enc_C)
    b_state = GPIO.input(Enc_D)
    
    # 將目前的A和B訊號組合成一個二進制數
    current_state = (a_state << 1) | b_state
    
    # 組合上一次的狀態和目前的狀態來判斷方向
    state_transition = (last_state_right << 2) | current_state
    
    # 根據狀態機的變化判斷方向
    if state_transition in [0b1101, 0b0100, 0b0010, 0b1011]:
        counter_right += 1
    elif state_transition in [0b1110, 0b0111, 0b0001, 0b1000]:
        counter_right -= 1
    
    # 更新最後狀態
    last_state_right = current_state

def calculate_wheel_speeds():
    global last_counter_left, last_counter_right
    current_counter_left = counter_left
    current_counter_right = counter_right
    
    delta_counter_left = current_counter_left - last_counter_left
    delta_counter_right = current_counter_right - last_counter_right
    
    last_counter_left = current_counter_left
    last_counter_right = current_counter_right

    # 計算左輪和右輪的角速度 (rad/s)
    angular_velocity_left = (delta_counter_left / (speed_sampling_time * 4000.0)) * 2 * pi  # 左輪弧度/秒
    angular_velocity_right = (delta_counter_right / (speed_sampling_time * 4000.0)) * 2 * pi  # 右輪弧度/秒

    # 計算左輪和右輪的線速度 (m/s)
    linear_velocity_left = angular_velocity_left * wheel_radius  # 公尺/秒
    linear_velocity_right = angular_velocity_right * wheel_radius  # 公尺/秒
    
    return linear_velocity_left, linear_velocity_right

def calculate_vehicle_speed():
    linear_velocity_left, linear_velocity_right = calculate_wheel_speeds()
    
    # 計算車體線速度 (v)
    linear_velocity = (linear_velocity_left + linear_velocity_right) / 2.0
    
    # 計算車體角速度 (ω)
    angular_velocity = (linear_velocity_right - linear_velocity_left) / wheel_base
    
    return linear_velocity, angular_velocity

def main():
    try:
        init()
        while True:
            sleep(speed_sampling_time)  # 每秒計算一次
            
            linear_velocity, angular_velocity = calculate_vehicle_speed()
            print(f"Vehicle Linear Velocity: {linear_velocity:.2f} m/s, Vehicle Angular Velocity: {angular_velocity:.2f} rad/s")

    except KeyboardInterrupt:
        GPIO.cleanup()

if __name__ == '__main__':
    main()
