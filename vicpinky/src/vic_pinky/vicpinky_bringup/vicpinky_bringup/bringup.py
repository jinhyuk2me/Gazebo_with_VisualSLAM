#!/usr/bin/env python3
"""
VicPinky 로봇 제어를 위한 ROS 2 노드
이 파일은 모터 드라이버와 ROS 2를 연결하여 로봇의 움직임을 제어합니다.
"""

import rclpy
from rclpy.node import Node
import math
import time

# ROS 2 메시지 타입들 - 로봇의 상태와 명령을 주고받기 위한 메시지들
from geometry_msgs.msg import Twist, TransformStamped  # Twist: 속도 명령, TransformStamped: 좌표 변환
from nav_msgs.msg import Odometry  # 로봇의 위치와 속도 정보
from sensor_msgs.msg import JointState  # 조인트(바퀴) 상태 정보
from tf2_ros import TransformBroadcaster  # 좌표계 변환 정보를 브로드캐스트
from tf_transformations import quaternion_from_euler  # 오일러 각을 쿼터니언으로 변환

# 모터 드라이버 모듈 - 실제 모터와 통신하는 저수준 드라이버
from vicpinky_bringup.zlac_driver import ZLACDriver

# --- 설정 상수들 ---
# ROS 2 토픽과 프레임 이름들
TWIST_SUB_TOPIC_NAME = "cmd_vel"  # 속도 명령을 받는 토픽 이름
ODOM_PUB_TOPIC_NAME = "odom"  # 오도메트리 정보를 발행하는 토픽 이름
JOINT_PUB_TOPIC_NAME = "joint_states"  # 조인트 상태를 발행하는 토픽 이름
ODOM_FRAME_ID = "odom"  # 오도메트리 좌표계 이름
ODOM_CHILD_FRAME_ID = "base_footprint"  # 로봇 베이스 좌표계 이름

# 시리얼 포트 설정
SERIAL_PORT_NAME = "/dev/motor"  # 모터 컨트롤러가 연결된 시리얼 포트
BAUDRATE = 115200  # 시리얼 통신 속도 (bps)
MODBUS_ID = 0x01  # 모터 컨트롤러의 Modbus 주소

# 로봇 조인트 이름들
JOINT_NAME_WHEEL_L = "left_wheel_joint"  # 왼쪽 바퀴 조인트 이름
JOINT_NAME_WHEEL_R = "right_wheel_joint"  # 오른쪽 바퀴 조인트 이름

# 로봇 물리적 사양
WHEEL_RAD = 0.0825  # 바퀴 반지름 (미터)
PULSE_PER_ROT = 4096  # 엔코더 1회전당 펄스 수
WHEEL_BASE = 0.475  # 바퀴 간격 (미터)
RPM2RAD = 0.104719755  # RPM을 rad/s로 변환하는 상수 (2π/60)
CIRCUMFERENCE = 2 * math.pi * WHEEL_RAD  # 바퀴 둘레 (미터)


class VicPinky(Node):
    """
    VicPinky 로봇을 제어하는 ROS 2 노드 클래스
    
    이 클래스는 다음과 같은 기능을 수행합니다:
    1. 모터 드라이버와 통신하여 로봇을 제어
    2. cmd_vel 토픽을 구독하여 속도 명령을 받음
    3. 오도메트리 정보를 계산하고 발행
    4. 조인트 상태 정보를 발행
    5. TF 변환 정보를 브로드캐스트
    """
    def __init__(self):
        # ROS 2 노드 초기화 - 노드 이름은 'vic_pinky_bringup'
        super().__init__('vic_pinky_bringup')
        self.is_initialized = False  # 초기화 성공 여부를 추적하는 플래그

        self.get_logger().info('Initializing Vic Pinky Bringup Node...')
        
        # 저수준 모터 드라이버 초기화
        # ZLACDriver: 실제 모터 컨트롤러와 시리얼 통신을 담당
        self.driver = ZLACDriver(SERIAL_PORT_NAME, BAUDRATE, MODBUS_ID)
        
        # --- 안정성을 위해 초기화 과정을 단계별로 실행 ---
        # 각 단계가 실패하면 노드 초기화를 중단하고 안전하게 종료
        
        # 1단계: 시리얼 포트 연결
        self.get_logger().info("1. Opening serial port...")
        if not self.driver.begin():
            self.get_logger().error("Failed to open serial port! Shutting down.")
            return  # __init__ 종료 - 더 이상 초기화를 진행하지 않음
        time.sleep(0.1)  # 하드웨어 안정화를 위한 짧은 대기

        # 2단계: 모터를 속도 제어 모드로 설정
        self.get_logger().info("2. Setting velocity mode...")
        if not self.driver.set_vel_mode():
            self.get_logger().error("Failed to set velocity mode! Shutting down.")
            self.driver.terminate()  # 시리얼 포트 정리
            return
        time.sleep(0.1)

        # 3단계: 모터 활성화
        self.get_logger().info("3. Enabling motors...")
        if not self.driver.enable():
            self.get_logger().error("Failed to enable motors! Shutting down.")
            self.driver.terminate()
            return
        
        # 모터 컨트롤러가 준비될 때까지 대기
        self.get_logger().info("Waiting for motor controller to be ready...")
        time.sleep(1.0)  # 모터 컨트롤러 초기화 완료 대기

        # 4단계: 모터 컨트롤러 응답성 확인
        self.get_logger().info("4. Verifying motor controller is responsive...")
        rpm_l, rpm_r = self.driver.get_rpm()  # 현재 RPM 값 읽기 시도
        if rpm_l is None:
            self.get_logger().error("Motor controller is not responding to status requests! Shutting down.")
            self.driver.terminate()
            return
        self.get_logger().info(f"Initial RPM read: L={rpm_l}, R={rpm_r}. Controller is responsive.")
        time.sleep(0.1)


        # 5단계: 모터 RPM을 0으로 초기화 (안전을 위해)
        self.get_logger().info("5. Setting initial RPM to zero (with retries)...")
        max_retries = 3  # 최대 재시도 횟수
        success = False
        for i in range(max_retries):
            self.get_logger().info(f"Attempt {i + 1}/{max_retries}...")
            if self.driver.set_double_rpm(0, 0):  # 왼쪽, 오른쪽 모터 모두 0 RPM으로 설정
                success = True
                break
            self.get_logger().warn(f"Attempt {i + 1} failed. Retrying in 0.2 seconds...")
            time.sleep(0.2)
        
        if not success:
            self.get_logger().error("Failed to set initial RPM after multiple retries! Shutting down.")
            self.driver.terminate()
            return
        
        # 6단계: 초기 엔코더 값 읽기
        # 오도메트리 계산을 위해 현재 엔코더 위치를 기준점으로 저장
        self.get_logger().info("6. Reading initial encoder values...")
        self.last_encoder_l, self.last_encoder_r = self.driver.get_position()
        if self.last_encoder_l is None or self.last_encoder_r is None:
            self.get_logger().error("Failed to read initial encoder position! Shutting down.")
            self.driver.terminate()
            return
            
        # ROS 2 퍼블리셔, 서브스크라이버, TF 브로드캐스터 생성
        # 퍼블리셔: 로봇의 상태 정보를 다른 노드들에게 전송
        self.odom_pub = self.create_publisher(Odometry, ODOM_PUB_TOPIC_NAME, 10)  # 오도메트리 발행
        self.joint_pub = self.create_publisher(JointState, JOINT_PUB_TOPIC_NAME, 10)  # 조인트 상태 발행
        
        # 서브스크라이버: 다른 노드로부터 명령을 받음
        self.twist_sub = self.create_subscription(Twist, TWIST_SUB_TOPIC_NAME, self.twist_callback, 10)  # 속도 명령 구독
        
        # TF 브로드캐스터: 좌표계 변환 정보를 전송
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # 타이머: 주기적으로 센서 데이터를 읽고 상태를 업데이트 (30Hz)
        self.timer = self.create_timer(1.0 / 30.0, self.update_and_publish)

        # 오도메트리 계산을 위한 변수들 초기화
        self.x = 0.0      # 로봇의 X 위치 (미터)
        self.y = 0.0      # 로봇의 Y 위치 (미터)
        self.theta = 0.0  # 로봇의 방향 (라디안)
        self.last_time = self.get_clock().now()  # 이전 업데이트 시간

        self.is_initialized = True  # 모든 초기화 성공 플래그 설정
        self.get_logger().info('Vic Pinky Bringup has been started successfully.')

    def twist_callback(self, msg: Twist):
        """
        cmd_vel 토픽으로부터 속도 명령을 받는 콜백 함수
        
        Twist 메시지를 받아서 차동 구동 로봇의 역기구학을 통해
        각 바퀴의 RPM으로 변환하여 모터에 전송합니다.
        
        Args:
            msg (Twist): 선속도(linear.x)와 각속도(angular.z)를 포함하는 메시지
        """
        linear_x = msg.linear.x    # 전진/후진 속도 (m/s)
        angular_z = msg.angular.z  # 회전 속도 (rad/s)

        # 역기구학(Inverse Kinematics): Twist 명령을 각 바퀴의 RPM으로 변환
        try:
            # 차동 구동 로봇의 역기구학 공식
            # 왼쪽 바퀴 속도 = 선속도 - (각속도 * 바퀴간격 / 2)
            # 오른쪽 바퀴 속도 = 선속도 + (각속도 * 바퀴간격 / 2)
            v_l = linear_x - (angular_z * WHEEL_BASE / 2.0)  # 왼쪽 바퀴 선속도
            v_r = linear_x + (angular_z * WHEEL_BASE / 2.0)  # 오른쪽 바퀴 선속도
            
            # 선속도를 RPM으로 변환
            # RPM = 선속도 / (바퀴반지름 * RPM2RAD_상수)
            rpm_l = v_l / (WHEEL_RAD * RPM2RAD)
            rpm_r = v_r / (WHEEL_RAD * RPM2RAD)
            
            # 모터의 최대 RPM 제한 적용 (-28 ~ 28 RPM)
            rpm_l = max(min(int(rpm_l), 28), -28)
            rpm_r = max(min(int(rpm_r), 28), -28)

            # 계산된 RPM을 모터 드라이버에 전송
            self.driver.set_double_rpm(rpm_l, rpm_r)
        except Exception as e:
            self.get_logger().warn(f"Failed to send motor data. rpm_l: {rpm_l}, rpm_r: {rpm_r}, error: {e}")


    def update_and_publish(self):
        """
        주기적으로 모터 데이터를 읽고, 오도메트리를 계산하며, 토픽들을 발행하는 함수
        
        이 함수는 타이머에 의해 30Hz로 호출되어 다음 작업을 수행합니다:
        1. 모터에서 현재 RPM과 엔코더 값을 읽음
        2. 엔코더 변화량으로부터 이동 거리 계산
        3. 오도메트리 업데이트 (위치와 방향)
        4. TF, Odometry, JointState 메시지 발행
        """
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9  # 시간 간격 (초)
        if dt <= 0:  # 시간이 역행하거나 0인 경우 스킵
            return

        # 모터 드라이버로부터 현재 상태 읽기
        rpm_l, rpm_r = self.driver.get_rpm()          # 현재 RPM
        encoder_l, encoder_r = self.driver.get_position()  # 현재 엔코더 위치

        if rpm_l is None or encoder_l is None:
            self.get_logger().warn("Failed to read motor data. Skipping this update cycle.")
            return

        # 엔코더 변화량으로부터 이동 거리 계산
        delta_l_pulses = encoder_l - self.last_encoder_l  # 왼쪽 바퀴 엔코더 변화량
        delta_r_pulses = encoder_r - self.last_encoder_r  # 오른쪽 바퀴 엔코더 변화량
        
        # 다음 계산을 위해 현재 엔코더 값 저장
        self.last_encoder_l = encoder_l
        self.last_encoder_r = encoder_r

        # 엔코더 펄스를 실제 이동 거리로 변환
        # 거리 = (펄스 변화량 / 1회전당 펄스수) * 바퀴 둘레
        dist_l = (delta_l_pulses / PULSE_PER_ROT) * CIRCUMFERENCE  # 왼쪽 바퀴 이동 거리
        dist_r = (delta_r_pulses / PULSE_PER_ROT) * CIRCUMFERENCE  # 오른쪽 바퀴 이동 거리

        # 차동 구동 로봇의 오도메트리 계산
        delta_distance = (dist_r + dist_l) / 2.0  # 로봇 중심의 이동 거리
        delta_theta = (dist_r - dist_l) / WHEEL_BASE  # 회전각 변화량
        self.theta += delta_theta  # 누적 회전각 업데이트

        # 업데이트된 방향을 기준으로 글로벌 좌표계에서의 위치 변화 계산
        d_x = delta_distance * math.cos(self.theta)  # X 방향 변위
        d_y = delta_distance * math.sin(self.theta)  # Y 방향 변위

        # 누적 위치 업데이트
        self.x += d_x
        self.y += d_y
        
        # 현재 속도 계산 (오도메트리 메시지용)
        v_x = delta_distance / dt    # 선속도 (m/s)
        vth = delta_theta / dt       # 각속도 (rad/s)

        # 각종 ROS 2 메시지 발행
        self._publish_tf(current_time)  # TF 변환 정보 발행
        self._publish_odometry(current_time, v_x, vth)  # 오도메트리 정보 발행
        self._publish_joint_states(current_time, vel_l_rads=rpm_l * RPM2RAD, vel_r_rads=rpm_r * RPM2RAD)  # 조인트 상태 발행

        self.last_time = current_time  # 다음 계산을 위해 현재 시간 저장

    def _publish_tf(self, current_time):
        """
        TF(Transform) 메시지를 발행하는 함수
        
        odom 좌표계에서 base_footprint 좌표계로의 변환 정보를 발행합니다.
        이를 통해 ROS 2의 tf2 시스템에서 로봇의 위치를 추적할 수 있습니다.
        
        Args:
            current_time: 현재 시간 (ROS 2 Time 객체)
        """
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()  # 타임스탬프 설정
        t.header.frame_id = ODOM_FRAME_ID       # 부모 좌표계 (odom)
        t.child_frame_id = ODOM_CHILD_FRAME_ID  # 자식 좌표계 (base_footprint)
        
        # 평행 이동 정보 설정
        t.transform.translation.x = self.x   # X 위치
        t.transform.translation.y = self.y   # Y 위치
        t.transform.translation.z = 0.0      # Z 위치 (2D 로봇이므로 0)
        
        # 회전 정보를 쿼터니언으로 변환하여 설정
        # 오일러 각 (roll=0, pitch=0, yaw=self.theta)을 쿼터니언으로 변환
        q = quaternion_from_euler(0, 0, self.theta)
        t.transform.rotation.x = q[0]  # 쿼터니언 x
        t.transform.rotation.y = q[1]  # 쿼터니언 y
        t.transform.rotation.z = q[2]  # 쿼터니언 z
        t.transform.rotation.w = q[3]  # 쿼터니언 w
        
        # TF 변환 정보를 브로드캐스트
        self.tf_broadcaster.sendTransform(t)

    def _publish_odometry(self, current_time, v_x, vth):
        """
        Odometry 메시지를 발행하는 함수
        
        로봇의 현재 위치, 방향, 속도 정보를 담은 Odometry 메시지를 발행합니다.
        Navigation 스택에서 로봇의 위치 추정에 사용됩니다.
        
        Args:
            current_time: 현재 시간 (ROS 2 Time 객체)
            v_x: 현재 선속도 (m/s)
            vth: 현재 각속도 (rad/s)
        """
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()  # 타임스탬프
        odom_msg.header.frame_id = ODOM_FRAME_ID        # 부모 좌표계
        odom_msg.child_frame_id = ODOM_CHILD_FRAME_ID   # 자식 좌표계
        
        # 위치 정보 설정
        odom_msg.pose.pose.position.x = self.x  # X 위치
        odom_msg.pose.pose.position.y = self.y  # Y 위치
        # Z 위치는 기본값 0 (2D 로봇)
        
        # 방향 정보를 쿼터니언으로 설정
        q = quaternion_from_euler(0, 0, self.theta)
        odom_msg.pose.pose.orientation.x = q[0]
        odom_msg.pose.pose.orientation.y = q[1]
        odom_msg.pose.pose.orientation.z = q[2]
        odom_msg.pose.pose.orientation.w = q[3]
        
        # 속도 정보 설정
        odom_msg.twist.twist.linear.x = v_x   # 선속도
        odom_msg.twist.twist.angular.z = vth  # 각속도
        # 다른 방향의 속도는 기본값 0 (차동 구동 로봇)
        
        # 공분산 행렬 설정 (불확실성 정보)
        # 36개 원소: 6x6 행렬 (x, y, z, roll, pitch, yaw)
        odom_msg.pose.covariance = [0.1] * 36    # 위치 불확실성
        odom_msg.twist.covariance = [0.1] * 36   # 속도 불확실성
        
        # Odometry 메시지 발행
        self.odom_pub.publish(odom_msg)

    def _publish_joint_states(self, current_time, vel_l_rads, vel_r_rads):
        """
        JointState 메시지를 발행하는 함수
        
        로봇의 각 조인트(바퀴)의 위치와 속도 정보를 발행합니다.
        RViz나 robot_state_publisher에서 로봇 모델의 시각화에 사용됩니다.
        
        Args:
            current_time: 현재 시간 (ROS 2 Time 객체)
            vel_l_rads: 왼쪽 바퀴의 각속도 (rad/s)
            vel_r_rads: 오른쪽 바퀴의 각속도 (rad/s)
        """
        joint_msg = JointState()
        joint_msg.header.stamp = current_time.to_msg()  # 타임스탬프
        joint_msg.name = [JOINT_NAME_WHEEL_L, JOINT_NAME_WHEEL_R]  # 조인트 이름들
        
        # 조인트 위치 계산 (엔코더 값을 라디안으로 변환)
        # 위치 = (엔코더 값 / 1회전당 펄스수) * 2π
        joint_msg.position = [
            (self.last_encoder_l / PULSE_PER_ROT) * (2 * math.pi),  # 왼쪽 바퀴 위치 (rad)
            (self.last_encoder_r / PULSE_PER_ROT) * (2 * math.pi)   # 오른쪽 바퀴 위치 (rad)
        ]
        
        # 조인트 속도 설정
        joint_msg.velocity = [vel_l_rads, vel_r_rads]  # 각속도 (rad/s)
        
        # JointState 메시지 발행
        self.joint_pub.publish(joint_msg)

    def on_shutdown(self):
        """
        노드 종료 시 호출되는 함수
        
        안전한 종료를 위해 모터를 정지시키고 드라이버 연결을 해제합니다.
        """
        self.get_logger().info("Shutting down, terminating motor driver...")
        
        # 모터를 안전하게 정지
        self.driver.set_double_rpm(0, 0)
        
        # 드라이버가 초기화되었는지 확인 후 종료
        if hasattr(self, 'driver') and self.driver:
            self.driver.terminate()


def main(args=None):
    """
    메인 함수 - 노드를 생성하고 실행합니다.
    
    Args:
        args: 명령행 인자들 (기본값: None)
    """
    # ROS 2 초기화
    rclpy.init(args=args)
    node = None
    
    try:
        # VicPinky 노드 생성
        node = VicPinky()
        
        # 노드가 성공적으로 초기화되었는지 확인
        if hasattr(node, 'is_initialized') and node.is_initialized:
            # 노드 실행 (무한 루프로 콜백 함수들을 처리)
            rclpy.spin(node)
            
    except KeyboardInterrupt:
        # Ctrl+C로 종료 시 정상적으로 처리
        pass
        
    finally:
        # 정리 작업
        if node:
            if hasattr(node, 'is_initialized') and node.is_initialized:
                node.on_shutdown()  # 안전한 종료 처리
                node.destroy_node()  # 노드 리소스 해제
                
        # ROS 2 종료
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    # 스크립트가 직접 실행될 때만 main() 함수 호출
    main()