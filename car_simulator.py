import rospy
import pygame
from geometry_msgs.msg import Twist

# 初始化 pygame
pygame.init()
screen = pygame.display.set_mode((800, 800))
pygame.display.set_caption("Joystick Simulator")
clock = pygame.time.Clock()

# 初始化joystick的位置
joystick_pos = [400, 400]  # joystick位置
joystick_speed = 0
joystick_angle = 0
joystick_radius = 20  # 定義joystick圓圈半徑

def callback(data):
    global joystick_speed, joystick_angle
    joystick_speed = data.linear.x
    joystick_angle = data.angular.z

def listener():
    rospy.init_node('joystick_simulator', anonymous=True)
    rospy.Subscriber('/amr_control_vel', Twist, callback)

    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                rospy.signal_shutdown("User requested shutdown")
                pygame.quit()
                return

        # 更新joystick的位置
        joystick_pos[1] -= int(joystick_speed * 10)
        joystick_pos[0] += int(joystick_angle * 10)  # 假設角速度影響 y 方向移動

        screen.fill((255, 255, 255))  # 填充背景色
        # 畫joystick初始位置(黑色填充圓形)
        pygame.draw.circle(screen, (0, 0, 0), (400, 400), 10)

        # 畫以joystick初始位置為中心的半径為100的圓圈
        pygame.draw.circle(screen, (0, 0, 0), (400, 400), 250, 3)
        pygame.draw.circle(screen, (0, 0, 255), joystick_pos, joystick_radius)  # 畫joystick
        pygame.display.flip()  # 更新顯示

        rate.sleep()
        clock.tick(60)

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
