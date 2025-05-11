import os
import sys
import rclpy
import csv
from turtlesim.msg import Pose
from rclpy.qos import QoSProfile
from datetime import datetime
import time  # time 모듈 임포트 추가
import select  # select 모듈 임포트 추가
import numpy as np
from geometry_msgs.msg import Twist

# 현재 시간 가져오기
current_time = datetime.now().strftime("%m%d_%H%M")

# CSV 파일 경로
CSV_FILE_PATH1 = f'robot1_poses_{current_time}.csv'
CSV_FILE_PATH2 = f'robot2_poses_{current_time}.csv'
CSV_FILE_PATH3 = f'robot3_poses_{current_time}.csv'
CSV_FILE_PATH4 = f'desired_poses_{current_time}.csv'
CSV_FILE_PATH5 = f'midpoint_poses_{current_time}.csv'

# CSV 파일에 헤더 작성
def initialize_csv(file_path):
    with open(file_path, mode='w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(['robot', 'time', 'x', 'y', 'theta', 'linear_velocity', 'angular_velocity'])

# 전역 변수
is_recording = False
start_time = None  # 시작 시간 초기화
base_angle = 0.0
theta_steer = 0.0
x = 0.0
y = 0.0
elapsed_time_pre = 0.0
lin_vel_pre = 0.0
ang_vel_pre = 0.0
base_angle_pre = 0.0


def pose_callback1(msg):
    global is_recording, start_time
    if is_recording:
        elapsed_time = (time.time() - start_time)  # 경과 시간 계산
        with open(CSV_FILE_PATH1, mode='a', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['turtle1', f'{elapsed_time:.3f}', msg.x, msg.y, msg.theta, msg.linear_velocity, msg.angular_velocity])
        print(f'Turtle1 - time: {elapsed_time:.3f}, x: {msg.x}, y: {msg.y}, theta: {msg.theta}, linear_velocity: {msg.linear_velocity}, angular_velocity: {msg.angular_velocity}')

def pose_callback2(msg):
    global is_recording, start_time
    if is_recording:
        elapsed_time = (time.time() - start_time)  # 경과 시간 계산
        with open(CSV_FILE_PATH2, mode='a', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['turtle2', f'{elapsed_time:.3f}', msg.x, msg.y, msg.theta, msg.linear_velocity, msg.angular_velocity])
        print(f'Turtle2 - time: {elapsed_time:.3f}, x: {msg.x}, y: {msg.y}, theta: {msg.theta}, linear_velocity: {msg.linear_velocity}, angular_velocity: {msg.angular_velocity}')

def pose_callback3(msg):
    global is_recording, start_time
    if is_recording:
        elapsed_time = (time.time() - start_time)  # 경과 시간 계산
        with open(CSV_FILE_PATH3, mode='a', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['turtle3', f'{elapsed_time:.3f}', msg.x, msg.y, msg.theta, msg.linear_velocity, msg.angular_velocity])
        print(f'Turtle3 - time: {elapsed_time:.3f}, x: {msg.x}, y: {msg.y}, theta: {msg.theta}, linear_velocity: {msg.linear_velocity}, angular_velocity: {msg.angular_velocity}')

def pose_callbackdes(msg):
    global is_recording, start_time, base_angle, theta_steer, x, y, elapsed_time_pre, lin_vel_pre, ang_vel_pre, base_angle_pre
    if is_recording:
        
        elapsed_time = (time.time() - start_time)  # 경과 시간 계산
        time_diff = elapsed_time - elapsed_time_pre
        dx = lin_vel_pre * np.cos(base_angle_pre + theta_steer + np.pi) * time_diff
        dy = lin_vel_pre * np.sin(base_angle_pre + theta_steer + np.pi) * time_diff
        x = x + dx
        y = y + dy
        base_angle_pre = base_angle_pre + ang_vel_pre * time_diff
    
        with open(CSV_FILE_PATH4, mode='a', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['desired_trajectory', f'{elapsed_time:.3f}', x, y, msg.angular.y, msg.linear.x, msg.angular.z])
        print(f'desired - time: {elapsed_time:.3f}, x: {x}, y: {y}, theta: {msg.angular.y}, linear_velocity: {msg.linear.x}, angular_velocity: {msg.angular.z}')
        elapsed_time_pre = elapsed_time
        lin_vel_pre = msg.linear.x
        ang_vel_pre = msg.angular.z
        theta_steer = msg.angular.y
        
        

def pose_callbackmid(msg):
    global is_recording, start_time, x, y, base_angle_pre
    if x == 0.0 and y == 0.0 :
        x = msg.x
        y = msg.y
        base_angle_pre = msg.angular_velocity
    if is_recording:
        elapsed_time = (time.time() - start_time)  # 경과 시간 계산
        with open(CSV_FILE_PATH5, mode='a', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['mid_point', f'{elapsed_time:.3f}', msg.x, msg.y, msg.theta, msg.linear_velocity, ang_vel_pre])
        print(f'mid_point - time: {elapsed_time:.3f}, x: {msg.x}, y: {msg.y}, theta: {msg.theta}, linear_velocity: {msg.linear_velocity}, angular_velocity: {ang_vel_pre}')

def get_key():
    """키 입력을 비동기적으로 받기 위한 함수"""
    if os.name == 'nt':
        import msvcrt
        return msvcrt.getch().decode('utf-8')
    else:
        import tty
        import termios
        settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setraw(sys.stdin.fileno())
            rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
            if rlist:
                key = sys.stdin.read(1)
            else:
                key = ''
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

def main():
    global is_recording, start_time

    rclpy.init()
    qos = QoSProfile(depth=10)
    node = rclpy.create_node('pose_logger')

    # Pose 구독
    # node.create_subscription(Pose, '/turtle1/pose', pose_callback1, qos)
    # node.create_subscription(Pose, '/turtle2/pose', pose_callback2, qos)
    # node.create_subscription(Pose, '/turtle3/pose', pose_callback3, qos)
    node.create_subscription(Pose, '/mid_point/pose', pose_callbackmid,qos)
    node.create_subscription(Twist, '/desired/cmd_vel', pose_callbackdes,qos)

    initialize_csv(CSV_FILE_PATH1)
    initialize_csv(CSV_FILE_PATH2)
    initialize_csv(CSV_FILE_PATH3)
    initialize_csv(CSV_FILE_PATH4)
    initialize_csv(CSV_FILE_PATH5)

    try:
        print("Press 's' to start recording and 'e' to stop recording.")
        while rclpy.ok():
            key = get_key()
            if key == 's':
                is_recording = True
                start_time = time.time()  # 시작 시간 기록
                print("Recording started.")
            elif key == 'e':
                is_recording = False
                print("Recording stopped.")
            rclpy.spin_once(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
