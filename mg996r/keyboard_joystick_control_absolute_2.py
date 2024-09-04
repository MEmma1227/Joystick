## 2024.09.04
import tty
import select  # I/O multiplexing module
import sys     # System-specific parameters and functions module
import time
import pygame
import pigpio

servo_X_pin = 18
servo_Y_pin = 12

pwm = pigpio.pi()
pwm.set_mode(servo_X_pin, pigpio.OUTPUT)
pwm.set_PWM_frequency(servo_X_pin, 50)  # 50Hz frequency
pwm.set_mode(servo_Y_pin, pigpio.OUTPUT)
pwm.set_PWM_frequency(servo_Y_pin, 50)  # 50Hz frequency

pygame.init()
screen = pygame.display.set_mode((800, 800))
pygame.display.set_caption("Joystick Simulator")
clock = pygame.time.Clock()

joystick_pos = [400, 400]
joystick_radius = 30

x_ini_pos = 0
y_ini_pos = 0
pos_bias = 0.5

# x_ini_angle = 80
x_ini_angle = 95
y_ini_angle = 110
level_bias = 15

x_pos = x_ini_pos
y_pos = y_ini_pos
x_angle = x_ini_angle
y_angle = y_ini_angle

def destroy():
    set_angle(servo_Y_pin, x_ini_angle)
    set_angle(servo_Y_pin, y_ini_angle)
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

    if key == '8': 
        x_pos = x_ini_pos
        y_pos = y_ini_pos + pos_bias
        x_angle = x_ini_angle 
        y_angle = y_ini_angle + level_bias
        set_angle(servo_Y_pin, y_angle)
        joystick_pos[0] = 400 + int(x_pos * 150)
        joystick_pos[1] = 400 - int(y_pos * 150)


    elif key == '2': 
        x_pos = x_ini_pos
        y_pos = y_ini_pos - pos_bias
        x_angle = x_ini_angle 
        y_angle = y_ini_angle - level_bias
        set_angle(servo_Y_pin, y_angle)
        joystick_pos[0] = 400 + int(x_pos * 150)
        joystick_pos[1] = 400 - int(y_pos * 150)

    elif key == '6':  
        x_pos = x_ini_pos + pos_bias
        y_pos = y_ini_pos 
        x_angle = x_ini_angle + level_bias
        y_angle = y_ini_angle 
        set_angle(servo_X_pin, x_angle)
        joystick_pos[0] = 400 + int(x_pos * 150)
        joystick_pos[1] = 400 - int(y_pos * 150)

    elif key == '4':   
        x_pos = x_ini_pos - pos_bias
        y_pos = y_ini_pos 
        x_angle = x_ini_angle - level_bias
        y_angle = y_ini_angle
        set_angle(servo_X_pin, x_angle)
        joystick_pos[0] = 400 + int(x_pos * 150)
        joystick_pos[1] = 400 - int(y_pos * 150)

    elif key == '5': 
        x_pos = x_ini_pos
        y_pos = y_ini_pos
        x_angle = x_ini_angle
        y_angle = y_ini_angle
        set_angle(servo_X_pin, x_angle)
        set_angle(servo_Y_pin, y_angle)
        joystick_pos[0] = 400 + int(x_pos * 150)
        joystick_pos[1] = 400 - int(y_pos * 150)

    elif key == '9': 
        x_pos = x_ini_pos + pos_bias
        y_pos = y_ini_pos + pos_bias
        x_angle = x_ini_angle + level_bias
        y_angle = y_ini_angle + level_bias
        set_angle(servo_X_pin, x_angle)
        set_angle(servo_Y_pin, y_angle)
        joystick_pos[0] = 400 + int(x_pos * 150)
        joystick_pos[1] = 400 - int(y_pos * 150)

    elif key == '7': 
        x_pos = x_ini_pos - pos_bias
        y_pos = y_ini_pos + pos_bias
        x_angle = x_ini_angle - level_bias
        y_angle = y_ini_angle + level_bias
        set_angle(servo_X_pin, x_angle)
        set_angle(servo_Y_pin, y_angle)
        joystick_pos[0] = 400 + int(x_pos * 150)
        joystick_pos[1] = 400 - int(y_pos * 150)

    elif key == '1': 
        x_pos = x_ini_pos - pos_bias
        y_pos = y_ini_pos - pos_bias
        x_angle = x_ini_angle - level_bias
        y_angle = y_ini_angle - level_bias
        set_angle(servo_X_pin, x_angle)
        set_angle(servo_Y_pin, y_angle)
        joystick_pos[0] = 400 + int(x_pos * 150)
        joystick_pos[1] = 400 - int(y_pos * 150)

    elif key == '3': 
        x_pos = x_ini_pos + pos_bias
        y_pos = y_ini_pos - pos_bias
        x_angle = x_ini_angle + level_bias
        y_angle = y_ini_angle - level_bias
        set_angle(servo_X_pin, x_angle)
        set_angle(servo_Y_pin, y_angle)
        joystick_pos[0] = 400 + int(x_pos * 150)
        joystick_pos[1] = 400 - int(y_pos * 150)
    # return x_pos, y_pos


def pygame_update():
    screen.fill((255, 255, 255))
    pygame.draw.circle(screen, (0, 0, 0), (400, 400), 10)
    pygame.draw.circle(screen, (0, 0, 0), (400, 400), 300, 3)
    pygame.draw.circle(screen, (0, 0, 255), joystick_pos, joystick_radius)
    pygame.display.flip()
    clock.tick(60)


def main():
    global pos_bias
    global level_bias
    key = get_key()
    while key != '-':
        if key == '*':
            pos_bias *= 2
            level_bias += 10
            if pos_bias > 1 :
                pos_bias = 1
            if level_bias > 25:
                level_bias = 25
        elif key == '/':
            pos_bias /= 2
            level_bias -= 10
            if pos_bias < 0.5 :
                pos_bias = 0.5
            if level_bias < 15:
                level_bias = 15

        keyboard_adjust_xypos()
        key = get_key()
        # print(key)
        pygame_update()

    destroy()

if __name__ == "__main__":
    main()

