import tty
import select  # I/O multiplexing module
import sys  # System-specific parameters and functions module

import pigpio
import time

servo_X_pin = 18
servo_Y_pin = 12

pwm = pigpio.pi()
pwm.set_mode(servo_X_pin, pigpio.OUTPUT)
pwm.set_PWM_frequency(servo_X_pin, 50)  # 50Hz frequency
pwm.set_mode(servo_Y_pin, pigpio.OUTPUT)
pwm.set_PWM_frequency(servo_Y_pin, 50)  # 50Hz frequency


x_pos = 0
y_pos = 0

x_ini_angle = 80
y_ini_angle = 110

x_angle = x_ini_angle
y_angle = y_ini_angle

def destroy():
    pwm.set_PWM_dutycycle(servo_X_pin, 0)
    pwm.set_PWM_frequency(servo_X_pin, 0)
    pwm.set_PWM_dutycycle(servo_Y_pin, 0)
    pwm.set_PWM_frequency(servo_Y_pin, 0)
    pwm.stop()

def set_angle(servo_pin, angle):
    # duty_cycle description
    # 0 = stop
    # 500 = 0 degree
    # 1500 = 90 degree
    # 2500 = 180 degree
    duty = 500 + (float(angle) / 180.0) * 2000  # python2  ^ ^   ^  ^  float   $
    pwm.set_servo_pulsewidth(servo_pin, duty)
    # print("(",x_angle, ",",y_angle,")  ", "GPIO ", servo_pin,": angle = ", ang$
    # time.sleep(0.5)

def get_key():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ""
    return key

def keyboard_adjust_xypos():
    global x_pos, y_pos
    global x_angle, y_angle
    global x_ini_angle, y_ini_angle

    key = get_key()

    if key == '8': #   ^  ^ ^ 
        #x_pos += 0
        y_pos += 1
        # x_angle += 0
        y_angle += 10
        set_angle(servo_Y_pin, y_angle)


    elif key == '2': #   ^   ^ 
        #x_pos += 0
        y_pos -= 1
        # x_angle += 0
        y_angle -= 10
        set_angle(servo_Y_pin, y_angle)

    elif key == '6': #   ^  ^  
        x_pos += 1
        #y_pos += 0
        x_angle += 10
        # y_angle += 0
        set_angle(servo_X_pin, x_angle)

    elif key == '4': #   ^    
        x_pos -= 1
        #y_pos += 0
        x_angle -= 10
        #y_angle += 0
        set_angle(servo_X_pin, x_angle)

    elif key == '5': #     ^  
        x_pos = 0
        y_pos = 0
        x_angle = x_ini_angle
        y_angle = y_ini_angle
        set_angle(servo_X_pin, x_angle)
        set_angle(servo_Y_pin, y_angle)

    # return x_pos, y_pos

def main():
    key = get_key()
    while key != '-':
        # x_pos, y_pos = keyboard_adjust_xypos()
        keyboard_adjust_xypos()
        #print((x_pos, y_pos))
        key = get_key()
        # if key == '\x03':
        #     twist.linear.x = 0.0
        #     twist.linear.y = 0.0
        #     twist.linear.z = 0.0
        #     twist.angular.x = 0.0
        #     twist.angular.y = 0.0
        #     twist.angular.z = 0.0
        #     pub.publish(twist)
        #     break
        # rate.sleep()

if __name__ == "__main__":
    main()

