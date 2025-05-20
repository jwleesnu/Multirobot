
#!/usr/bin/env python
#
# Copyright (c) 2011, Willow Garage, Inc.
# All rights reserved.
#
# Software License Agreement (BSD License 2.0)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of {copyright_holder} nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Darby Lim

import os
import select
import sys
import rclpy
import time
import threading
import numpy as np

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16, Bool
from rclpy.qos import QoSProfile
from turtlesim.msg import Pose

if os.name == 'nt':
    import msvcrt
else:
    import termios
    import tty

BURGER_MAX_LIN_VEL = 100#0.22/2
BURGER_MAX_ANG_VEL = 100#2.84/2

WAFFLE_MAX_LIN_VEL = 100#0.26/2
WAFFLE_MAX_ANG_VEL = 100#1.82/2

LIN_VEL_STEP_SIZE = 0.02
ANG_VEL_STEP_SIZE = 0.02

TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']

msg = """
Control Your TurtleBot3!
---------------------------
Moving around:                     Motors:
        w                               i
   a    s    d                          k
        x                               m

w/x : increase/decrease linear velocity (Burger : ~ 0.22, Waffle and Waffle Pi : ~ 0.26)
a/d : increase/decrease angular velocity (Burger : ~ 2.84, Waffle and Waffle Pi : ~ 1.82)
i/k/m : move lift upwards/stop/downwards
1/2/3 : controlling 1st robot, 2nd robot, all

space key, s : force stop

CTRL-C to quit
"""

e = """
Communications Failed
"""
height_count = 0
height_change_running = False
motor_status = "stop"
pwm = 0
x1 = 0.0
y1 = 0.0
theta1 = 0.0
x2 = 1.0
y2 = 0.0
theta2 = 0.0
x3 = 1.0
y3 = 0.0
theta3 = 0.0

def get_key(settings):
    if os.name == 'nt':
        return msvcrt.getch().decode('utf-8')
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def change_height():
    global height_count, height_change_running, motor_status, pwm
    while height_change_running:
        time.sleep(0.05)
        if motor_status == "cw" and height_count < 255:
            height_count += 1
        elif motor_status == "ccw" and height_count > 0:
            height_count -= 1
        elif height_count >=255 or height_count <=0:
            height_change_running = False
            pwm = 0
            
        
        # 바 형태로 출력
        bar_length = 50  # 바의 길이
        filled_length = int(bar_length * height_count / 255)  # 채워진 길이 비율
        bar = '█' * filled_length + '-' * (bar_length - filled_length)  # 바 생성
        print(f'\rHeight Count: [{bar}] {height_count}/255', end='')  # 출력 업데이트
        sys.stdout.flush()  # 버퍼 비우기



def print_vels(target_linear_velocity, target_angular_velocity,motor_status):
    print('\ncurrently:\tlinear velocity {0}\t angular velocity {1}\t motor command {2}'.format(
        target_linear_velocity,
        target_angular_velocity,
        motor_status))
    sys.stdout.flush()


def make_simple_profile(output, input, slop):
    if input > output:
        output = min(input, output + slop)
    elif input < output:
        output = max(input, output - slop)
    else:
        output = input

    return output


def constrain(input_vel, low_bound, high_bound):
    if input_vel < low_bound:
        input_vel = low_bound
    elif input_vel > high_bound:
        input_vel = high_bound
    else:
        input_vel = input_vel

    return input_vel


def check_linear_limit_velocity(velocity):
    if TURTLEBOT3_MODEL == 'burger':
        return constrain(velocity, -BURGER_MAX_LIN_VEL, BURGER_MAX_LIN_VEL)
    else:
        return constrain(velocity, -WAFFLE_MAX_LIN_VEL, WAFFLE_MAX_LIN_VEL)
    


def check_angular_limit_velocity(velocity):
    if TURTLEBOT3_MODEL == 'burger':
        return constrain(velocity, -BURGER_MAX_ANG_VEL, BURGER_MAX_ANG_VEL)
    else:
        return constrain(velocity, -WAFFLE_MAX_ANG_VEL, WAFFLE_MAX_ANG_VEL)

def pose_callback(msg):
    #print('callbacked',msg.x,' ',msg.y)
    global x1, y1, theta1
    x1 = msg.x
    y1 = msg.y
    theta1 = msg.theta + np.pi

def pose_callback2(msg):
    #print('callbacked',msg.x, ' ',msg.y)
    global x2, y2, theta2
    x2 = msg.x
    y2 = msg.y
    theta2 = msg.theta + np.pi

def pose_callback3(msg):
    #print('callbacked',msg.x, ' ',msg.y, msg.theta)
    global x3, y3, theta3
    x3 = msg.x
    y3 = msg.y
    theta3 = msg.theta + np.pi

# def odom_callback1(msg):
#     x1 = msg.pose.pose.position.x
#     y1 = msg.pose.pose.position.y
#     theta1 = Quattoyaw(msg.pose.pose.orientation)

def main():
    global motor_status,height_change_running,pwm,x1,y1,theta1,x2,y2,theta2# 전역 변수 사용 선언
    
    settings = None
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    rclpy.init()
    ###### for real
    qos = QoSProfile(depth=10)
    node = rclpy.create_node('teleop_keyboard')
    pub1 = node.create_publisher(Twist, '/turtle1/cmd_vel', qos)
    pubcw1 = node.create_publisher(Bool, '/robot1/gpio_output_27', qos)
    pubpwm1 = node.create_publisher(Int16, '/robot1/gpio_pwm_17', qos)
    pub2 = node.create_publisher(Twist, '/turtle2/cmd_vel', qos)
    pubcw2 = node.create_publisher(Bool, '/robot2/gpio_output_27', qos)
    pubpwm2 = node.create_publisher(Int16, '/robot2/gpio_pwm_17', qos)
    ###### for simulation
    pub3 = node.create_publisher(Twist, '/turtle3/cmd_vel', qos)
    pubmid = node.create_publisher(Pose,'/mid_point/pose',qos)
    pubdes = node.create_publisher(Twist, '/desired/cmd_vel',qos)
    ######
    qossub1 = QoSProfile(depth=10)
    qossub2 = QoSProfile(depth=10)
    qossub3 = QoSProfile(depth=10)
    qosodom1 = QoSProfile(depth=10)
    node.create_subscription(Pose, '/turtle1/pose', pose_callback, qossub1)
    node.create_subscription(Pose, '/turtle2/pose', pose_callback2, qossub2)
    node.create_subscription(Pose, '/turtle3/pose', pose_callback3, qossub3)
    # node.create_subscription(Odometry,'/robot1/odom', odom_callback1, qosodom1)

    status = 0
    target_linear_velocity = 0.0
    target_angular_velocity = 0.0
    control_linear_velocity = 0.0
    control_angular_velocity = 0.0
    cw_flag = True  # cw
    control_first = True
    control_second = False
    l = 10.0
    R1 = 0.0
    R2 = 0.0
    theta_steer = 0.0

    try:
        print(msg)
        
        while( rclpy.ok() ):
            #print(x1,x2,y1,y2)
            key = get_key(settings)
            if key == 'w':
                target_linear_velocity = check_linear_limit_velocity(target_linear_velocity + LIN_VEL_STEP_SIZE)
                status += 1
                print_vels(target_linear_velocity, target_angular_velocity, motor_status)
            elif key == 'x':
                target_linear_velocity = check_linear_limit_velocity(target_linear_velocity - LIN_VEL_STEP_SIZE)
                status += 1
                print_vels(target_linear_velocity, target_angular_velocity, motor_status)
            elif key == 'a':
                target_angular_velocity = check_angular_limit_velocity(target_angular_velocity + ANG_VEL_STEP_SIZE)
                status += 1
                print_vels(target_linear_velocity, target_angular_velocity, motor_status)
            elif key == 'd':
                target_angular_velocity = check_angular_limit_velocity(target_angular_velocity - ANG_VEL_STEP_SIZE)
                status += 1
                print_vels(target_linear_velocity, target_angular_velocity, motor_status)
            if key == '1':
                control_first = True
                control_second = False
                print('controlling first robot')
            elif key == '2':
                control_first = False
                control_second = True
                print('controlling second robot')
            elif key == '3':
                control_first = True
                control_second = True
                xmid = (x2+x1)/2
                ymid = (y2+y1)/2
                l = np.linalg.norm(np.array([x2-x1,y2-y1]))
                l3 = np.linalg.norm(np.array([x3 - xmid , y3 - ymid]))
                pose_theta3 = np.arctan2(y3 - ymid, x3 -xmid ) - np.arctan2(y2-y1,x2-x1)
                x3_rel = l3 * np.cos(pose_theta3 )
                y3_rel = l3 * np.sin(pose_theta3 )
                print('controlling both robots, l =',l,'l3=',l3,'pose_theta3 = ',pose_theta3 / np.pi ,'pi')
            elif key == 'q':
                theta_steer += 0.01 * np.pi
                #print('theta_steer = ', theta_steer)
            elif key == 'e':
                theta_steer -= 0.01 * np.pi
                #print('theta_steer = ', theta_steer)
            elif key == ' ' or key == 's':
                target_linear_velocity = 0.0
                control_linear_velocity = 0.0
                target_angular_velocity = 0.0
                control_angular_velocity = 0.0
                pwm = 0
                motor_status = "stop"
                height_change_running = False
                print_vels(target_linear_velocity, target_angular_velocity, motor_status)
            elif key == 'i':
                if height_count < 255:
                    cw_flag = True
                    pwm = 128
                    motor_status = "cw"
                    height_change_running = True
                    threading.Thread(target=change_height).start()
                    print_vels(target_linear_velocity, target_angular_velocity, motor_status)
                else:
                    pwm = 0
                    motor_status = "stop"
                    print_vels(target_linear_velocity, target_angular_velocity, motor_status)
                    print("!!!height upper limit reached!!!")
            elif key == 'k':
                pwm = 0
                motor_status = "stop"
                height_change_running = False
                print_vels(target_linear_velocity, target_angular_velocity, motor_status)
            elif key == 'm':
                if height_count > 0:
                    cw_flag = False
                    pwm = 128
                    motor_status = "ccw"
                    height_change_running = True
                    threading.Thread(target=change_height).start()
                    print_vels(target_linear_velocity, target_angular_velocity, motor_status)
                else:
                    pwm = 0
                    motor_status = "stop"
                    print_vels(target_linear_velocity, target_angular_velocity, motor_status)
                    print("!!!height lower limit reached!!!")
            else:
                if (key == '\x03'):
                    break

            if status == 20:
                print(msg)
                status = 0

            #control_linear_velocity1 = 0.0
            #control_linear_velocity2 = 0.0
            if control_first and control_second: # constraints should be defined
                base_angle = np.arctan2(y2-y1,x2 - x1)
                #print(base_angle/np.pi, 'pi')
                
                control_linear_velocity = target_linear_velocity
                control_angular_velocity = target_angular_velocity
                # control_linear_velocity = make_simple_profile(
                #     control_linear_velocity,
                #     target_linear_velocity,
                #     (LIN_VEL_STEP_SIZE / 2.0))
                # control_angular_velocity = make_simple_profile(
                #     control_angular_velocity,
                #     target_angular_velocity,
                #     (ANG_VEL_STEP_SIZE / 2.0))
                dt = 1 / 9.5
                # theta_steer_acc += control_angular_velocity * dt

                # theta_steer_acc = theta_steer_acc % (2*np.pi)
                # while (theta_steer_acc < 0) : 
                #     theta_steer_acc += 2*np.pi
                #theta_steer = theta_steer_acc - base_angle
                theta_steer = theta_steer % (2*np.pi)
                while (theta_steer < 0) : 
                    theta_steer += 2*np.pi
                # if control_angular_velocity > 0 :
                #     psi = theta_steer - np.pi / 2
                # else :
                #     psi = theta_steer + np.pi / 2
                #print(theta_steer_acc)
                #print(base_angle)
                print(f'theta_steer = {theta_steer/np.pi:.2f}pi')


                if np.abs(control_angular_velocity) < 1e-2:
                    target_linear_velocity1 = control_linear_velocity
                    target_linear_velocity2 = control_linear_velocity
                    target_theta1 = theta_steer
                    target_theta2 = theta_steer
                    ###
                    target_linear_velocity3 = control_linear_velocity
                    target_theta3 = theta_steer

                elif np.abs(control_angular_velocity) > 1e-2 and np.abs(control_linear_velocity) < 1e-2 :
                    if(control_angular_velocity > 0):
                        target_theta1 = np.pi/2 
                        target_theta2 = 3*np.pi/2 
                        target_linear_velocity1 = l/2 * control_angular_velocity
                        target_linear_velocity2 = l/2 * control_angular_velocity
                        target_theta3 = pose_theta3 - np.pi/2
                        target_linear_velocity3 = l3 * control_angular_velocity
                        
                    else:
                        target_theta1 = 3*np.pi/2 
                        target_theta2 = np.pi/2 
                        target_linear_velocity1 = -l/2 * control_angular_velocity
                        target_linear_velocity2 = -l/2 * control_angular_velocity
                        target_theta3 = pose_theta3 + np.pi/2
                        target_linear_velocity3 = -l3 * control_angular_velocity
                    

                else:
                    print('control_angular_velocity =',control_angular_velocity)

                    if ((control_angular_velocity < 0 and ((theta_steer >= 0 and theta_steer < np.pi/2) or (theta_steer <= 2*np.pi and theta_steer > 3/2*np.pi)) and control_linear_velocity > 0) or\
                        (control_angular_velocity > 0 and ((theta_steer >= 0 and theta_steer < np.pi/2) or (theta_steer <= 2*np.pi and theta_steer > 3/2*np.pi)) and control_linear_velocity < 0)): #R_D
                        print('R_D')#OK
                        rho = -control_linear_velocity / control_angular_velocity
                        psi = theta_steer - np.pi/2
                        x = rho * np.cos(psi)
                        y = rho * np.sin(psi)
                        target_theta1 = np.arctan2(y, x - l/2) + 1/2 * np.pi  
                        target_theta2 = np.arctan2(y, x + l/2) + 3/2 * np.pi
                        target_linear_velocity1 = -control_angular_velocity * l / np.sin(target_theta2 - target_theta1) * np.cos(target_theta2)
                        target_linear_velocity2 = control_angular_velocity * l / np.sin(target_theta2 - target_theta1) * np.cos(target_theta1)
                        target_theta2 +=np.pi
                        r3 = np.linalg.norm([x + x3_rel,y + y3_rel])
                        target_theta3 = np.arctan2(y + y3_rel,x + x3_rel) + np.pi/2
                        target_linear_velocity3 = -r3 * control_angular_velocity
                        

                    if ((control_angular_velocity > 0 and (theta_steer > np.pi/2 and theta_steer < 3/2 * np.pi) and control_linear_velocity > 0) or\
                        (control_angular_velocity < 0 and (theta_steer > np.pi/2 and theta_steer < 3/2 * np.pi) and control_linear_velocity < 0)): #L_D
                        print('L_D')
                        rho = control_linear_velocity / control_angular_velocity
                        psi = theta_steer + np.pi/2
                        x = rho * np.cos(psi)
                        y = rho * np.sin(psi)
                        target_theta1 = np.arctan2(y, x - l/2) - 1/2 * np.pi 
                        target_theta2 = np.arctan2(y, x + l/2) - 1/2 * np.pi
                        target_linear_velocity1 = -control_angular_velocity * l / np.sin(target_theta2 - target_theta1) * np.cos(target_theta2)
                        target_linear_velocity2 = -control_angular_velocity * l / np.sin(target_theta2 - target_theta1) * np.cos(target_theta1)
                        r3 = np.linalg.norm([x + x3_rel,y + y3_rel])
                        target_theta3 = np.arctan2(y + y3_rel,x + x3_rel) - np.pi/2
                        target_linear_velocity3 = r3 * control_angular_velocity


                    if ((control_angular_velocity > 0 and ((theta_steer >= 0 and theta_steer < np.pi/2) or (theta_steer <= 2*np.pi and theta_steer > 3/2*np.pi)) and control_linear_velocity > 0) or\
                        (control_angular_velocity < 0 and ((theta_steer >= 0 and theta_steer < np.pi/2) or (theta_steer <= 2*np.pi and theta_steer > 3/2*np.pi)) and control_linear_velocity < 0)): #L_U
                        print('L_U')#OK
                        rho = control_linear_velocity / control_angular_velocity
                        psi = theta_steer + np.pi/2
                        x = rho * np.cos(psi)
                        y = rho * np.sin(psi)
                        target_theta1 = np.arctan2(y, x - l/2) - 1/2 * np.pi 
                        target_theta2 = np.arctan2(y, x + l/2) - 1/2 * np.pi
                        target_linear_velocity1 = -control_angular_velocity * l / np.sin(target_theta2 - target_theta1) * np.cos(target_theta2)
                        target_linear_velocity2 = -control_angular_velocity * l / np.sin(target_theta2 - target_theta1) * np.cos(target_theta1)
                        r3 = np.linalg.norm([x + x3_rel,y + y3_rel])
                        target_theta3 = np.arctan2(y + y3_rel,x + x3_rel) - np.pi/2
                        target_linear_velocity3 = r3 * control_angular_velocity


                    if ((control_angular_velocity > 0 and (theta_steer > np.pi/2 and theta_steer < 3/2 * np.pi) and control_linear_velocity < 0) or\
                        (control_angular_velocity < 0 and (theta_steer > np.pi/2 and theta_steer < 3/2 * np.pi) and control_linear_velocity > 0)): #R_U
                        print('R_U')
                        rho = -control_linear_velocity / control_angular_velocity
                        psi = theta_steer - np.pi/2
                        x = rho * np.cos(psi)
                        y = rho * np.sin(psi)
                        target_theta1 = np.arctan2(y, x - l/2) + 1/2 * np.pi 
                        target_theta2 = np.arctan2(y, x + l/2) + 1/2 * np.pi
                        target_linear_velocity1 = -control_angular_velocity * l / np.sin(target_theta2 - target_theta1) * np.cos(target_theta2)
                        target_linear_velocity2 = -control_angular_velocity * l / np.sin(target_theta2 - target_theta1) * np.cos(target_theta1)
                        r3 = np.linalg.norm([x + x3_rel,y + y3_rel])
                        target_theta3 = np.arctan2(y + y3_rel,x + x3_rel) + np.pi/2
                        target_linear_velocity3 = -r3 * control_angular_velocity
                    #target_theta3 = np.arctan2(y - (l3 * np.sin(pose_theta3)), x - (l3 * np.cos(pose_theta3))) + np.pi/2
                    #target_linear_velocity3 = -control_angular_velocity * np.linalg.norm(np.array([l3 * np.cos(pose_theta3) - x , l3 * np.sin(pose_theta3) - y]))
                    # print('x = ',x, 'y = ',y)
                    # print('rho = ',rho, 'psi =',psi/np.pi,'pi ', 'target_theta1 =', target_theta1/np.pi,'pi ','target_theta2 = ',target_theta2/np.pi,'pi ','target_linear_velocity1 = ',target_linear_velocity1,'target_linear_velocity2 = ',target_linear_velocity2)
                # print('target_theta3',target_theta3/np.pi,'pi','vel_theta3',target_linear_velocity3)
                K_p = 2

                twist1 = Twist()
                twist2 = Twist()
                twist3 = Twist()
                twist1.linear.x = target_linear_velocity1
                twist1.linear.y = 0.0
                twist1.linear.z = 0.0
                twist2.linear.x = target_linear_velocity2
                twist2.linear.y = 0.0
                twist2.linear.z = 0.0
                twist3.linear.x = target_linear_velocity3
                twist3.linear.y = 0.0
                twist3.linear.z = 0.0
                #print( 'target', (target_theta1 + base_angle + 2.5 / l *control_angular_velocity )/np.pi,'pi')
                angle_diff1 = (target_theta1 + base_angle + 2.5 / l *control_angular_velocity - theta1) % (2 * np.pi)
    
                # 음수 값 처리
                if angle_diff1 < 0:
                    angle_diff1 += 2 * np.pi  # 음수일 경우 2π를 더해줍니다.

                # 각도 차이를 -π에서 π로 조정
                if angle_diff1 > np.pi:
                    angle_diff1 -= 2 * np.pi

                twist1.angular.x = 0.0
                twist1.angular.y = 0.0
                twist1.angular.z = K_p * angle_diff1

                angle_diff2 = (target_theta2 + base_angle + 2.5 / l * control_angular_velocity - theta2) % (2 * np.pi)
                
                if angle_diff2 < 0:
                    angle_diff2 += 2 * np.pi  
                if angle_diff2 > np.pi:
                    angle_diff2 -= 2 * np.pi

                twist2.angular.x = 0.0
                twist2.angular.y = 0.0
                twist2.angular.z = K_p * angle_diff2

                angle_diff3 = (target_theta3 + base_angle + 2.5 / l3 * control_angular_velocity - theta3) % (2 * np.pi)
    
                if angle_diff3 < 0:
                    angle_diff3 += 2 * np.pi  

                if angle_diff3 > np.pi:
                    angle_diff3 -= 2 * np.pi

                twist3.angular.x = 0.0
                twist3.angular.y = 0.0
                twist3.angular.z = K_p * angle_diff3

                twistdes = Twist()
                twistdes.linear.x = control_linear_velocity
                twistdes.linear.y = 0.0
                twistdes.linear.z = 0.0
                twistdes.angular.x = base_angle #####
                twistdes.angular.y = theta_steer ###
                twistdes.angular.z = control_angular_velocity

                ####simulation
                posemid = Pose()
                posemid.x = (x1+x2)/2
                posemid.y = (y1+y2)/2
                posemid.theta = theta_steer
                posemid.linear_velocity = control_linear_velocity
                posemid.angular_velocity = control_angular_velocity #base_angle
                ####
                pub1.publish(twist1)
                pub2.publish(twist2)
                pub3.publish(twist3)
                pubmid.publish(posemid)
                pubdes.publish(twistdes)
                
            
            else:
                twist = Twist()
                control_linear_velocity = make_simple_profile(
                    control_linear_velocity,
                    target_linear_velocity,
                    (LIN_VEL_STEP_SIZE / 2.0))

                twist.linear.x = control_linear_velocity
                twist.linear.y = 0.0
                twist.linear.z = 0.0

                control_angular_velocity = make_simple_profile(
                    control_angular_velocity,
                    target_angular_velocity,
                    (ANG_VEL_STEP_SIZE / 2.0))

                twist.angular.x = 0.0
                twist.angular.y = 0.0
                twist.angular.z = control_angular_velocity
                if control_first == True and control_second == False:
                    pub1.publish(twist)
                
                    cw_msg = Bool()
                    cw_msg.data = cw_flag
                    pubcw1.publish(cw_msg)

                    pwm_msg = Int16()
                    pwm_msg.data = pwm
                    pubpwm1.publish(pwm_msg)
                if control_second == True and control_first == False:
                    pub2.publish(twist)
                
                    cw_msg = Bool()
                    cw_msg.data = cw_flag
                    pubcw2.publish(cw_msg)

                    pwm_msg = Int16()
                    pwm_msg.data = pwm
                    pubpwm2.publish(pwm_msg)
            rclpy.spin_once(node)
            




    except Exception as e:
        print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0

        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0

        pub1.publish(twist)
        pub2.publish(twist)
        
        pwm_msg = Int16()
        pwm_msg.data = 0
        pubpwm1.publish(pwm_msg)
        pubpwm2.publish(pwm_msg)

        if os.name != 'nt':
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    
    

if __name__ == '__main__':
    main()
