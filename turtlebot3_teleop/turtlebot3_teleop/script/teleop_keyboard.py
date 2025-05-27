#!/usr/bin/env python3
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
import yaml
from ament_index_python.packages import get_package_share_directory

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

class Turtle:
    def __init__(self, node, name, is_real=False):
        self.name = name
        self.is_real = is_real
        self.node = node
        self.qos = QoSProfile(depth=10)
        
        # Initialize state variables
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        self.target_linear_velocity = 0.0
        self.target_angular_velocity = 0.0
        self.target_theta = 0.0
        self.K_p = 2 # proportional gain for angle
        
        self.l = 0.0    # distance to the midpoint
        self.pose_theta = 0.0    # angle to the midpoint
        
        # Create publishers
        self.cmd_vel_pub = node.create_publisher(Twist, f'/{name}/cmd_vel', self.qos)
        if is_real:
            self.cw_pub = node.create_publisher(Bool, f'/robot{name[-1]}/gpio_output_27', self.qos)
            self.pwm_pub = node.create_publisher(Int16, f'/robot{name[-1]}/gpio_pwm_17', self.qos)
        
        # Create subscriber
        self.pose_sub = node.create_subscription(
            Pose, f'/{name}/pose', 
            lambda msg: self.pose_callback(msg), 
            self.qos
        )

    def set_target(self, linear_vel, target_theta, base_angle, control_angular_velocity):
        self.target_linear_velocity = linear_vel
        self.target_theta = target_theta + base_angle + 0.5 * control_angular_velocity
        angle_diff = (self.target_theta - self.theta) % (2 * np.pi)
        if angle_diff < 0:
            angle_diff += 2 * np.pi
        if angle_diff > np.pi:
            angle_diff -= 2 * np.pi
        self.target_angular_velocity = self.K_p * angle_diff

    def pose_callback(self, msg):
        self.x = msg.x
        self.y = msg.y
        self.theta = msg.theta + np.pi

    def publish_velocity(self):
        twist = Twist()
        twist.linear.x = self.target_linear_velocity
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = self.target_angular_velocity
        self.cmd_vel_pub.publish(twist)

    def publish_motor_control(self, cw_flag, pwm):
        if self.is_real:
            cw_msg = Bool()
            cw_msg.data = cw_flag
            self.cw_pub.publish(cw_msg)

            pwm_msg = Int16()
            pwm_msg.data = pwm
            self.pwm_pub.publish(pwm_msg)

class TurtleController:
    def __init__(self):
        self.node = rclpy.create_node('teleop_keyboard')
        self.qos = QoSProfile(depth=10)
        
        # Load turtle configurations from config.yaml using ROS2 package system
        try:
            turtlesim_share_dir = get_package_share_directory('turtlesim')
            config_path = os.path.join(turtlesim_share_dir, 'config', 'config.yaml')
            with open(config_path, 'r') as f:
                config = yaml.safe_load(f)
            self.node.get_logger().info(f'Loaded config from: {config_path}')
        except Exception as e:
            self.node.get_logger().error(f'Failed to load config: {str(e)}')
            # Fallback to default configuration
            config = {
                'turtles': {
                    'turtle1': {'x': 400, 'y': 500, 'theta': 0.0},
                    'turtle2': {'x': 300, 'y': 300, 'theta': 0.0},
                    'turtle3': {'x': 200, 'y': 500, 'theta': 0.0}
                }
            }
            self.node.get_logger().info('Using default configuration')
        
        # Create turtles list
        self.turtles = []
        # Add turtles based on config
        for turtle_name in config['turtles'].keys():
            is_real = turtle_name in ['turtle1', 'turtle2']  # First two turtles are real robots
            self.turtles.append(Turtle(self.node, turtle_name, is_real))
            self.node.get_logger().info(f'Created turtle: {turtle_name} (real: {is_real})')
        
        # Create midpoint publisher
        self.midpoint_pub = self.node.create_publisher(Pose, '/mid_point/pose', self.qos)
        self.desired_pub = self.node.create_publisher(Twist, '/desired/cmd_vel', self.qos)
        
        # Control state
        self.control_first = True
        self.control_second = False
        self.control_all = False
        self.control_linear_velocity = 0.0
        self.control_angular_velocity = 0.0
        self.theta_steer = 0.0
        
        # Initialize state tracking variables
        self.fixed_l = None  # Fixed l value when '3' is pressed
        self.last_base_angle = 0.0
        
        # Motor control
        self.height_count = 0
        self.height_change_running = False
        self.motor_status = "stop"
        self.pwm = 0
        self.cw_flag = True

    def get_key(self, settings):
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

    def change_height(self):
        while self.height_change_running:
            time.sleep(0.05)
            if self.motor_status == "cw" and self.height_count < 255:
                self.height_count += 1
            elif self.motor_status == "ccw" and self.height_count > 0:
                self.height_count -= 1
            elif self.height_count >= 255 or self.height_count <= 0:
                self.height_change_running = False
                self.pwm = 0
            
            bar_length = 50
            filled_length = int(bar_length * self.height_count / 255)
            bar = '█' * filled_length + '-' * (bar_length - filled_length)
            print(f'\rHeight Count: [{bar}] {self.height_count}/255', end='')
            sys.stdout.flush()

    def print_vels(self):
        print('\ncurrently:\tlinear velocity {0}\t angular velocity {1}\t motor command {2}'.format(
            self.control_linear_velocity,
            self.control_angular_velocity,
            self.motor_status))
        sys.stdout.flush()

    def run(self):
        settings = None
        if os.name != 'nt':
            settings = termios.tcgetattr(sys.stdin)

        try:
            print(self.get_help_message())
            while rclpy.ok():
                key = self.get_key(settings)
                self.process_key(key)
                self.update_turtles()
                rclpy.spin_once(self.node)

        except Exception as e:
            print(e)
        finally:
            self.stop_all()

    def get_help_message(self):
        return """
Control Your TurtleBot3!
---------------------------
Moving around:                     Motors:
        w                               i
   a    s    d                          k
        x                               m

w/x : increase/decrease linear velocity
a/d : increase/decrease angular velocity
i/k/m : move lift upwards/stop/downwards
1/2/3 : controlling 1st robot, 2nd robot, all

space key, s : force stop

CTRL-C to quit
"""

    def process_key(self, key):
        if key == 'w':
            self.control_linear_velocity = self.check_linear_limit_velocity(
                self.control_linear_velocity + LIN_VEL_STEP_SIZE)
            self.print_vels()
        elif key == 'x':
            self.control_linear_velocity = self.check_linear_limit_velocity(
                self.control_linear_velocity - LIN_VEL_STEP_SIZE)
            self.print_vels()
        elif key == 'a':
            self.control_angular_velocity = self.check_angular_limit_velocity(
                self.control_angular_velocity + ANG_VEL_STEP_SIZE)
            self.print_vels()
        elif key == 'd':
            self.control_angular_velocity = self.check_angular_limit_velocity(
                self.control_angular_velocity - ANG_VEL_STEP_SIZE)
            self.print_vels()
        elif key == '1':
            self.control_first = True
            self.control_second = False
            self.control_all = False
            print('controlling first robot')
        elif key == '2':
            self.control_first = False
            self.control_second = True
            self.control_all = False
            print('controlling second robot')
        elif key == '3':
            self.control_first = True
            self.control_second = True
            self.control_all = True
            # Reset fixed_l to force recalculation
            self.fixed_l = None
            xmid, ymid = self.update_midpoint()
            for i in range(len(self.turtles)):
                self.turtles[i].x_rel = self.turtles[i].x - xmid
                self.turtles[i].y_rel = self.turtles[i].y - ymid
                self.turtles[i].pose_theta = np.arctan2(self.turtles[i].y_rel, self.turtles[i].x_rel)
        elif key == 'q':
            self.theta_steer += 0.01 * np.pi
        elif key == 'e':
            self.theta_steer -= 0.01 * np.pi
        elif key == ' ' or key == 's':
            self.stop_all()
        elif key == 'i':
            self.handle_motor_up()
        elif key == 'k':
            self.handle_motor_stop()
        elif key == 'm':
            self.handle_motor_down()
        elif key == '\x03':
            raise KeyboardInterrupt

    def update_midpoint(self):
        # Calculate midpoint between first two turtles
        xmid = (self.turtles[1].x + self.turtles[0].x) / 2
        ymid = (self.turtles[1].y + self.turtles[0].y) / 2
        
        # Only calculate l if it hasn't been fixed yet
        if self.fixed_l is None:
            self.fixed_l = np.linalg.norm(np.array([self.turtles[1].x - self.turtles[0].x, 
                                                  self.turtles[1].y - self.turtles[0].y]))
            # Calculate relative positions for all turtles
        for i in range(len(self.turtles)):
            self.turtles[i].l = np.linalg.norm(np.array([self.turtles[i].x - xmid, 
                                                        self.turtles[i].y - ymid]))
            self.turtles[i].pose_theta = np.arctan2(self.turtles[i].y - ymid, 
                                                    self.turtles[i].x - xmid) - \
                                        np.arctan2(self.turtles[1].y - self.turtles[0].y, 
                                                    self.turtles[1].x - self.turtles[0].x)
            
            if i >= 2:  # For additional turtles
                print(f'Turtle {i+1} - l{i+1}={self.turtles[i].l:.2f}, pose_theta{i+1}={self.turtles[i].pose_theta/np.pi:.2f}pi')
        
            print('controlling all robots, l =', self.fixed_l)
        return xmid, ymid

    def update_turtles(self):
        if self.control_all:
            _,_=self.update_midpoint()
            self.update_all_control()
        else:
            self.update_single_control()

    def update_all_control(self):
        # Calculate base angle and velocities
        base_angle = np.arctan2(self.turtles[1].y - self.turtles[0].y, 
                              self.turtles[1].x - self.turtles[0].x)
        
        # Normalize theta_steer
        self.theta_steer = self.theta_steer % (2 * np.pi)
        while self.theta_steer < 0:
            self.theta_steer += 2 * np.pi
            
        print(f'theta_steer = {self.theta_steer/np.pi:.2f}pi')
        
        # Calculate and publish velocities
        self.calculate_target_velocities(
            self.control_linear_velocity,
            self.control_angular_velocity,
            base_angle
        )
        
        # Publish velocities
        for turtle in self.turtles:
            turtle.publish_velocity()
        
        # Publish midpoint and desired velocities
        self.publish_midpoint_and_desired(
            self.control_linear_velocity,
            self.control_angular_velocity,
            base_angle
        )

    def calculate_target_velocities(self, control_linear_velocity, control_angular_velocity, base_angle):
        if np.abs(control_angular_velocity) < 1e-2:
            # Straight line motion
            for i in range(len(self.turtles)):
                self.turtles[i].set_target(control_linear_velocity, self.theta_steer, base_angle, control_angular_velocity)
                
        elif np.abs(control_linear_velocity) < 1e-2 and np.abs(control_angular_velocity) > 1e-2:
            # Pure rotation
            if control_angular_velocity > 0:
                self.turtles[0].set_target(self.fixed_l / 2 * control_angular_velocity, np.pi/2, base_angle, control_angular_velocity)
                self.turtles[1].set_target(self.fixed_l / 2 * control_angular_velocity, 3*np.pi/2, base_angle, control_angular_velocity)
                for i in range(2, len(self.turtles)):
                    self.turtles[i].set_target(
                        self.turtles[i].l * control_angular_velocity,
                        self.turtles[i].pose_theta - np.pi/2,
                        base_angle,
                        control_angular_velocity
                    )
            else:
                self.turtles[0].set_target(-self.fixed_l / 2 * control_angular_velocity, 3*np.pi/2, base_angle, control_angular_velocity)
                self.turtles[1].set_target(-self.fixed_l / 2 * control_angular_velocity, np.pi/2, base_angle, control_angular_velocity)
                for i in range(2, len(self.turtles)):
                    self.turtles[i].set_target(
                        -self.turtles[i].l * control_angular_velocity,
                        self.turtles[i].pose_theta + np.pi/2,
                        base_angle,
                        control_angular_velocity
                    )
        else:
            print('control_angular_velocity =', control_angular_velocity)
            
            if ((control_angular_velocity < 0 and ((self.theta_steer >= 0 and self.theta_steer < np.pi/2) or (self.theta_steer <= 2*np.pi and self.theta_steer > 3/2*np.pi)) and control_linear_velocity > 0) or\
                (control_angular_velocity > 0 and ((self.theta_steer >= 0 and self.theta_steer < np.pi/2) or (self.theta_steer <= 2*np.pi and self.theta_steer > 3/2*np.pi)) and control_linear_velocity < 0)): #R_D
                print('R_D')#OK
                rho = -control_linear_velocity / control_angular_velocity
                psi = self.theta_steer - np.pi/2
                x = rho * np.cos(psi)
                y = rho * np.sin(psi)
                target_theta1 = np.arctan2(y, x - self.turtles[0].l) + np.pi/2
                target_theta2 = np.arctan2(y, x + self.turtles[1].l) + 3*np.pi/2
                target_vel1 = -control_angular_velocity * (self.fixed_l) / np.sin(target_theta2 - target_theta1) * np.cos(target_theta2)
                target_vel2 = control_angular_velocity * (self.fixed_l) / np.sin(target_theta2 - target_theta1) * np.cos(target_theta1)
                target_theta2 += np.pi
                
                self.turtles[0].set_target(target_vel1, target_theta1, base_angle, control_angular_velocity)
                self.turtles[1].set_target(target_vel2, target_theta2, base_angle, control_angular_velocity)
                
                for i in range(2, len(self.turtles)):
                    ri = np.linalg.norm(np.array([
                        self.turtles[i].l * np.cos(self.turtles[i].pose_theta) + x,
                        self.turtles[i].l * np.sin(self.turtles[i].pose_theta) + y
                    ]))
                    target_thetai = np.arctan2(
                        self.turtles[i].l * np.sin(self.turtles[i].pose_theta) + y,
                        self.turtles[i].l * np.cos(self.turtles[i].pose_theta) + x
                    ) + np.pi/2
                    target_veli = -ri * control_angular_velocity
                    self.turtles[i].set_target(target_veli, target_thetai, base_angle, control_angular_velocity)

            elif ((control_angular_velocity > 0 and (self.theta_steer > np.pi/2 and self.theta_steer < 3/2 * np.pi) and control_linear_velocity > 0) or\
                (control_angular_velocity < 0 and (self.theta_steer > np.pi/2 and self.theta_steer < 3/2 * np.pi) and control_linear_velocity < 0)): #L_D
                print('L_D')
                rho = control_linear_velocity / control_angular_velocity
                psi = self.theta_steer + np.pi/2
                x = rho * np.cos(psi)
                y = rho * np.sin(psi)
                target_theta1 = np.arctan2(y, x - self.turtles[0].l) - np.pi/2
                target_theta2 = np.arctan2(y, x + self.turtles[1].l) - np.pi/2
                target_vel1 = -control_angular_velocity * (self.fixed_l) / np.sin(target_theta2 - target_theta1) * np.cos(target_theta2)
                target_vel2 = -control_angular_velocity * (self.fixed_l) / np.sin(target_theta2 - target_theta1) * np.cos(target_theta1)
                
                self.turtles[0].set_target(target_vel1, target_theta1, base_angle, control_angular_velocity)
                self.turtles[1].set_target(target_vel2, target_theta2, base_angle, control_angular_velocity)
                
                for i in range(2, len(self.turtles)):
                    ri = np.linalg.norm(np.array([
                        self.turtles[i].l * np.cos(self.turtles[i].pose_theta) + x,
                        self.turtles[i].l * np.sin(self.turtles[i].pose_theta) + y
                    ]))
                    target_thetai = np.arctan2(
                        self.turtles[i].l * np.sin(self.turtles[i].pose_theta) + y,
                        self.turtles[i].l * np.cos(self.turtles[i].pose_theta) + x
                    ) - np.pi/2
                    target_veli = ri * control_angular_velocity
                    self.turtles[i].set_target(target_veli, target_thetai, base_angle, control_angular_velocity)

            elif ((control_angular_velocity > 0 and ((self.theta_steer >= 0 and self.theta_steer < np.pi/2) or (self.theta_steer <= 2*np.pi and self.theta_steer > 3/2*np.pi)) and control_linear_velocity > 0) or\
                (control_angular_velocity < 0 and ((self.theta_steer >= 0 and self.theta_steer < np.pi/2) or (self.theta_steer <= 2*np.pi and self.theta_steer > 3/2*np.pi)) and control_linear_velocity < 0)): #L_U
                print('L_U')#OK
                rho = control_linear_velocity / control_angular_velocity
                psi = self.theta_steer + np.pi/2
                x = rho * np.cos(psi)
                y = rho * np.sin(psi)
                target_theta1 = np.arctan2(y, x - self.turtles[0].l) - np.pi/2
                target_theta2 = np.arctan2(y, x + self.turtles[1].l) - np.pi/2
                target_vel1 = -control_angular_velocity * (self.fixed_l) / np.sin(target_theta2 - target_theta1) * np.cos(target_theta2)
                target_vel2 = -control_angular_velocity * (self.fixed_l) / np.sin(target_theta2 - target_theta1) * np.cos(target_theta1)
                
                self.turtles[0].set_target(target_vel1, target_theta1, base_angle, control_angular_velocity)
                self.turtles[1].set_target(target_vel2, target_theta2, base_angle, control_angular_velocity)
                
                for i in range(2, len(self.turtles)):
                    ri = np.linalg.norm(np.array([
                        self.turtles[i].l * np.cos(self.turtles[i].pose_theta) + x,
                        self.turtles[i].l * np.sin(self.turtles[i].pose_theta) + y
                    ]))
                    target_thetai = np.arctan2(
                        self.turtles[i].l * np.sin(self.turtles[i].pose_theta) + y,
                        self.turtles[i].l * np.cos(self.turtles[i].pose_theta) + x
                    ) - np.pi/2
                    target_veli = ri * control_angular_velocity
                    self.turtles[i].set_target(target_veli, target_thetai, base_angle, control_angular_velocity)

            elif ((control_angular_velocity > 0 and (self.theta_steer > np.pi/2 and self.theta_steer < 3/2 * np.pi) and control_linear_velocity < 0) or\
                (control_angular_velocity < 0 and (self.theta_steer > np.pi/2 and self.theta_steer < 3/2 * np.pi) and control_linear_velocity > 0)): #R_U
                print('R_U')
                rho = -control_linear_velocity / control_angular_velocity
                psi = self.theta_steer - np.pi/2
                x = rho * np.cos(psi)
                y = rho * np.sin(psi)
                target_theta1 = np.arctan2(y, x - self.turtles[0].l) + np.pi/2
                target_theta2 = np.arctan2(y, x + self.turtles[1].l) + np.pi/2
                target_vel1 = -control_angular_velocity * (self.fixed_l) / np.sin(target_theta2 - target_theta1) * np.cos(target_theta2)
                target_vel2 = -control_angular_velocity * (self.fixed_l) / np.sin(target_theta2 - target_theta1) * np.cos(target_theta1)
                
                self.turtles[0].set_target(target_vel1, target_theta1, base_angle, control_angular_velocity)
                self.turtles[1].set_target(target_vel2, target_theta2, base_angle, control_angular_velocity)
                
                for i in range(2, len(self.turtles)):
                    ri = np.linalg.norm(np.array([
                        self.turtles[i].l * np.cos(self.turtles[i].pose_theta) + x,
                        self.turtles[i].l * np.sin(self.turtles[i].pose_theta) + y
                    ]))
                    target_thetai = np.arctan2(
                        self.turtles[i].l * np.sin(self.turtles[i].pose_theta) + y,
                        self.turtles[i].l * np.cos(self.turtles[i].pose_theta) + x
                    ) + np.pi/2
                    target_veli = -ri * control_angular_velocity
                    self.turtles[i].set_target(target_veli, target_thetai, base_angle, control_angular_velocity)

    def publish_midpoint_and_desired(self, lin_vel, ang_vel, base_angle):
        # Publish midpoint pose
        midpoint = Pose()
        midpoint.x = (self.turtles[1].x + self.turtles[0].x) / 2
        midpoint.y = (self.turtles[1].y + self.turtles[0].y) / 2
        midpoint.theta = self.theta_steer
        midpoint.linear_velocity = lin_vel
        midpoint.angular_velocity = ang_vel
        self.midpoint_pub.publish(midpoint)
        
        # Publish desired velocities
        desired = Twist()
        desired.linear.x = lin_vel
        desired.linear.y = 0.0
        desired.linear.z = 0.0
        desired.angular.x = base_angle
        desired.angular.y = self.theta_steer
        desired.angular.z = ang_vel
        self.desired_pub.publish(desired)

    def update_single_control(self):

        if self.control_first:
            self.turtles[0].target_linear_velocity = self.control_linear_velocity
            self.turtles[0].target_angular_velocity = self.control_angular_velocity
            self.turtles[0].publish_velocity()
            self.turtles[0].publish_motor_control(self.cw_flag, self.pwm)
        if self.control_second:
            self.turtles[1].target_linear_velocity = self.control_linear_velocity
            self.turtles[1].target_angular_velocity = self.control_angular_velocity
            self.turtles[1].publish_velocity()
            self.turtles[1].publish_motor_control(self.cw_flag, self.pwm)

    def stop_all(self):
        self.control_linear_velocity = 0.0
        self.control_angular_velocity = 0.0
        self.pwm = 0
        self.motor_status = "stop"
        self.height_change_running = False
        self.print_vels()

    def handle_motor_up(self):
        if self.height_count < 255:
            self.cw_flag = True
            self.pwm = 128
            self.motor_status = "cw"
            self.height_change_running = True
            threading.Thread(target=self.change_height).start()
            self.print_vels()
        else:
            self.pwm = 0
            self.motor_status = "stop"
            self.print_vels()
            print("!!!height upper limit reached!!!")

    def handle_motor_stop(self):
        self.pwm = 0
        self.motor_status = "stop"
        self.height_change_running = False
        self.print_vels()

    def handle_motor_down(self):
        if self.height_count > 0:
            self.cw_flag = False
            self.pwm = 128
            self.motor_status = "ccw"
            self.height_change_running = True
            threading.Thread(target=self.change_height).start()
            self.print_vels()
        else:
            self.pwm = 0
            self.motor_status = "stop"
            self.print_vels()
            print("!!!height lower limit reached!!!")

    @staticmethod
    def check_linear_limit_velocity(velocity):
        if TURTLEBOT3_MODEL == 'burger':
            return constrain(velocity, -BURGER_MAX_LIN_VEL, BURGER_MAX_LIN_VEL)
        else:
            return constrain(velocity, -WAFFLE_MAX_LIN_VEL, WAFFLE_MAX_LIN_VEL)

    @staticmethod
    def check_angular_limit_velocity(velocity):
        if TURTLEBOT3_MODEL == 'burger':
            return constrain(velocity, -BURGER_MAX_ANG_VEL, BURGER_MAX_ANG_VEL)
        else:
            return constrain(velocity, -WAFFLE_MAX_ANG_VEL, WAFFLE_MAX_ANG_VEL)

def constrain(input_vel, low_bound, high_bound):
    if input_vel < low_bound:
        input_vel = low_bound
    elif input_vel > high_bound:
        input_vel = high_bound
    return input_vel

def main():
    rclpy.init()
    controller = TurtleController()
    controller.run()

if __name__ == '__main__':
    main()
