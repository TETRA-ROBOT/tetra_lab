#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from interfaces.msg import Gpio
from interfaces.srv import PowerSetOutport, ConveyorAutoMovement
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from herkulex.srv import PanAngleMove, TiltAngleMove
import math


class GpioControllerNode(Node):
    def __init__(self):
        super().__init__('gpio_controller_node')
        
        # 이전 GPIO 상태 저장
        self.prev_output0 = None
        self.prev_output1 = None
        self.prev_output2 = None
        self.prev_output3 = None
        self.prev_output4 = None
        self.prev_output5 = None
        self.prev_output6 = None
        self.prev_output7 = None
        
        # GPIO status 구독
        self.gpio_sub = self.create_subscription(
            Gpio,
            'gpio_status',
            self.gpio_callback,
            10
        )
        
        # Power outport 서비스 클라이언트
        self.power_outport_client = self.create_client(
            PowerSetOutport,
            'Power_outport_cmd'
        )
        
        # Follow Joint Trajectory 액션 클라이언트
        self.trajectory_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/joint_trajectory_controller/follow_joint_trajectory'
        )
        
        # Pan/Tilt 제어 서비스 클라이언트
        self.pan_move_client = self.create_client(
            PanAngleMove,
            'PanMove_cmd'
        )
        
        self.tilt_move_client = self.create_client(
            TiltAngleMove,
            'TiltMove_cmd'
        )
        
        # Conveyor 제어 서비스 클라이언트
        self.conveyor_auto_move_client = self.create_client(
            ConveyorAutoMovement,
            'Conveyor_auto_move_cmd'
        )
        
        self.get_logger().info('GPIO Controller Node 시작됨')
    
    def gpio_callback(self, msg):
        """GPIO 상태 변화 감지 및 처리"""
        # output0이 0에서 1로 변경 감지 (Conveyor start=0)
        if self.prev_output0 is not None and self.prev_output0 == 0 and msg.output0 == 1:
            self.get_logger().info('output0이 1로 변경됨 - Conveyor start=0')
            self.handle_output0_change(msg)
        
        # output1이 0에서 1로 변경 감지 (Conveyor start=8)
        if self.prev_output1 is not None and self.prev_output1 == 0 and msg.output1 == 1:
            self.get_logger().info('output1이 1로 변경됨 - Conveyor start=8')
            self.handle_output1_change(msg)
        
        # output2이 0에서 1로 변경 감지 (Pan 90도)
        if self.prev_output2 is not None and self.prev_output2 == 0 and msg.output2 == 1:
            self.get_logger().info('output2가 1로 변경됨 - Pan 90도 회전')
            self.handle_output2_change(msg)
        
        # output3이 0에서 1로 변경 감지 (Pan 0도)
        if self.prev_output3 is not None and self.prev_output3 == 0 and msg.output3 == 1:
            self.get_logger().info('output3이 1로 변경됨 - Pan 0도')
            self.handle_output3_change(msg)
        
        # output4가 0에서 1로 변경 감지 (Tilt 80도)
        if self.prev_output4 is not None and self.prev_output4 == 0 and msg.output4 == 1:
            self.get_logger().info('output4가 1로 변경됨 - Tilt 80도')
            self.handle_output4_change(msg)
        
        # output5가 0에서 1로 변경 감지 (Tilt 0도)
        if self.prev_output5 is not None and self.prev_output5 == 0 and msg.output5 == 1:
            self.get_logger().info('output5가 1로 변경됨 - Tilt 0도')
            self.handle_output5_change(msg)
        
        # output6이 0에서 1로 변경 감지
        if self.prev_output6 is not None and self.prev_output6 == 0 and msg.output6 == 1:
            self.get_logger().info('output6이 1로 변경됨')
            self.handle_output6_change(msg)
        
        # output7이 0에서 1로 변경 감지
        if self.prev_output7 is not None and self.prev_output7 == 0 and msg.output7 == 1:
            self.get_logger().info('output7이 1로 변경됨')
            self.handle_output7_change(msg)
        
        # 현재 상태 저장
        self.prev_output0 = msg.output0
        self.prev_output1 = msg.output1
        self.prev_output2 = msg.output2
        self.prev_output3 = msg.output3
        self.prev_output4 = msg.output4
        self.prev_output5 = msg.output5
        self.prev_output6 = msg.output6
        self.prev_output7 = msg.output7
    
    def handle_output0_change(self, gpio_msg):
        """output0이 1로 변경 시 처리: Conveyor start=0"""
        self.get_logger().info('Conveyor Auto Movement start=0 전송')
        
        # Conveyor_auto_move_cmd 서비스 호출
        self.call_conveyor_auto_move(0)
        
        # Power_outport_cmd로 output0을 0으로 설정
        self.set_output_port(
            output0=0,  # output0을 0으로 설정
            output1=gpio_msg.output1,
            output2=gpio_msg.output2,
            output3=gpio_msg.output3,
            output4=gpio_msg.output4,
            output5=gpio_msg.output5,
            output6=gpio_msg.output6,
            output7=gpio_msg.output7
        )
    
    def handle_output1_change(self, gpio_msg):
        """output1이 1로 변경 시 처리: Conveyor start=8"""
        self.get_logger().info('Conveyor Auto Movement start=8 전송')
        
        # Conveyor_auto_move_cmd 서비스 호출
        self.call_conveyor_auto_move(8)
        
        # Power_outport_cmd로 output1을 0으로 설정
        self.set_output_port(
            output0=gpio_msg.output0,
            output1=0,  # output1을 0으로 설정
            output2=gpio_msg.output2,
            output3=gpio_msg.output3,
            output4=gpio_msg.output4,
            output5=gpio_msg.output5,
            output6=gpio_msg.output6,
            output7=gpio_msg.output7
        )
    
    def handle_output2_change(self, gpio_msg):
        """output2가 1로 변경 시 처리: Pan 90도 회전"""
        self.get_logger().info('Pan을 90도로 회전 시작')
        
        # PanMove_cmd 서비스 호출
        self.call_pan_move(90.0, 100)  # 90도, playtime 100 (1초)
        
        # Power_outport_cmd로 output2를 0으로 설정
        self.set_output_port(
            output0=gpio_msg.output0,
            output1=gpio_msg.output1,
            output2=0,  # output2를 0으로 설정
            output3=gpio_msg.output3,
            output4=gpio_msg.output4,
            output5=gpio_msg.output5,
            output6=gpio_msg.output6,
            output7=gpio_msg.output7
        )
    
    def handle_output3_change(self, gpio_msg):
        """output3이 1로 변경 시 처리: Pan 0도"""
        self.get_logger().info('Pan을 0도로 이동 시작')
        
        # PanMove_cmd 서비스 호출
        self.call_pan_move(0.0, 100)  # 0도, playtime 100 (1초)
        
        # Power_outport_cmd로 output3을 0으로 설정
        self.set_output_port(
            output0=gpio_msg.output0,
            output1=gpio_msg.output1,
            output2=gpio_msg.output2,
            output3=0,  # output3을 0으로 설정
            output4=gpio_msg.output4,
            output5=gpio_msg.output5,
            output6=gpio_msg.output6,
            output7=gpio_msg.output7
        )
    
    def handle_output4_change(self, gpio_msg):
        """output4가 1로 변경 시 처리: Tilt 80도"""
        self.get_logger().info('Tilt을 80도로 이동 시작')
        
        # TiltMove_cmd 서비스 호출
        self.call_tilt_move(80.0, 100)  # 80도, playtime 100 (1초)
        
        # Power_outport_cmd로 output4를 0으로 설정
        self.set_output_port(
            output0=gpio_msg.output0,
            output1=gpio_msg.output1,
            output2=gpio_msg.output2,
            output3=gpio_msg.output3,
            output4=0,  # output4를 0으로 설정
            output5=gpio_msg.output5,
            output6=gpio_msg.output6,
            output7=gpio_msg.output7
        )
    
    def handle_output5_change(self, gpio_msg):
        """output5가 1로 변경 시 처리: Tilt 0도"""
        self.get_logger().info('Tilt을 0도로 이동 시작')
        
        # TiltMove_cmd 서비스 호출
        self.call_tilt_move(0.0, 100)  # 0도, playtime 100 (1초)
        
        # Power_outport_cmd로 output5를 0으로 설정
        self.set_output_port(
            output0=gpio_msg.output0,
            output1=gpio_msg.output1,
            output2=gpio_msg.output2,
            output3=gpio_msg.output3,
            output4=gpio_msg.output4,
            output5=0,  # output5를 0으로 설정
            output6=gpio_msg.output6,
            output7=gpio_msg.output7
        )
    
    def handle_output6_change(self, gpio_msg):
        """output6 변화 시 처리: 모든 조인트를 0으로 이동"""
        self.get_logger().info('모든 조인트를 0으로 이동 시작')
        
        # 조인트 이름
        joint_names = ['base', 'shoulder', 'elbow', 'wrist1', 'wrist2', 'wrist3']
        
        # 목표 위치: 모든 조인트 0도 (라디안)
        positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        # 액션 실행
        self.send_trajectory(joint_names, positions, 6.0)
        
        # Power_outport_cmd로 output6을 0으로 설정
        self.set_output_port(
            output0=gpio_msg.output0,
            output1=gpio_msg.output1,
            output2=gpio_msg.output2,
            output3=gpio_msg.output3,
            output4=gpio_msg.output4,
            output5=gpio_msg.output5,
            output6=0,  # output6을 0으로 설정
            output7=gpio_msg.output7
        )
    
    def handle_output7_change(self, gpio_msg):
        """output7이 1로 변경 시 처리: shoulder 80도, elbow -150도"""
        self.get_logger().info('shoulder 80도, elbow -150도로 이동 시작')
        
        # 조인트 이름
        joint_names = ['base', 'shoulder', 'elbow', 'wrist1', 'wrist2', 'wrist3']
        
        # 목표 위치: shoulder 80도, elbow -150도 (도 -> 라디안 변환)
        positions = [
            0.0,                           # base
            math.radians(80.0),            # shoulder: 80도
            math.radians(-150.0),          # elbow: -150도
            0.0,                           # wrist1
            0.0,                           # wrist2
            0.0                            # wrist3
        ]
        
        # 액션 실행
        self.send_trajectory(joint_names, positions, 6.0)
        
        # Power_outport_cmd로 output7을 0으로 설정
        self.set_output_port(
            output0=gpio_msg.output0,
            output1=gpio_msg.output1,
            output2=gpio_msg.output2,
            output3=gpio_msg.output3,
            output4=gpio_msg.output4,
            output5=gpio_msg.output5,
            output6=gpio_msg.output6,
            output7=0  # output7을 0으로 설정
        )
    
    def send_trajectory(self, joint_names, positions, duration_sec):
        """Joint Trajectory 액션 전송"""
        # 액션 서버 대기
        if not self.trajectory_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('액션 서버를 찾을 수 없습니다!')
            return
        
        # Goal 메시지 생성
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = joint_names
        
        # Trajectory Point 생성
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = Duration(sec=int(duration_sec), nanosec=0)
        
        goal_msg.trajectory.points.append(point)
        
        # 액션 전송
        self.get_logger().info(f'액션 전송: {joint_names}')
        self.get_logger().info(f'위치: {positions}')
        
        send_goal_future = self.trajectory_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        """액션 Goal 응답 콜백"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal이 거부되었습니다!')
            return
        
        self.get_logger().info('Goal이 수락되었습니다.')
        
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        """액션 결과 콜백"""
        result = future.result().result
        self.get_logger().info(f'액션 완료: error_code={result.error_code}')
    
    def set_output_port(self, output0, output1, output2, output3, 
                        output4, output5, output6, output7):
        """Power_outport_cmd 서비스 호출"""
        if not self.power_outport_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Power_outport_cmd 서비스를 찾을 수 없습니다!')
            return
        
        # 서비스 요청 생성
        request = PowerSetOutport.Request()
        request.output0 = output0
        request.output1 = output1
        request.output2 = output2
        request.output3 = output3
        request.output4 = output4
        request.output5 = output5
        request.output6 = output6
        request.output7 = output7
        
        # 서비스 호출
        self.get_logger().info(f'Power_outport_cmd 서비스 호출')
        future = self.power_outport_client.call_async(request)
        future.add_done_callback(self.service_response_callback)
    
    def service_response_callback(self, future):
        """서비스 응답 콜백"""
        try:
            response = future.result()
            if response.command_result:
                self.get_logger().info('Power_outport_cmd 서비스 호출 성공')
            else:
                self.get_logger().error('Power_outport_cmd 서비스 호출 실패')
        except Exception as e:
            self.get_logger().error(f'서비스 호출 중 오류: {str(e)}')
    
    def call_pan_move(self, target_angle, playtime):
        """PanMove_cmd 서비스 호출"""
        if not self.pan_move_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('PanMove_cmd 서비스를 찾을 수 없습니다!')
            return
        
        # 서비스 요청 생성
        request = PanAngleMove.Request()
        request.target_angle = float(target_angle)
        request.playtime = int(playtime)
        
        # 서비스 호출
        self.get_logger().info(f'PanMove_cmd 서비스 호출 (angle={target_angle}도, playtime={playtime})')
        future = self.pan_move_client.call_async(request)
        future.add_done_callback(self.pan_tilt_response_callback)
    
    def call_tilt_move(self, target_angle, playtime):
        """TiltMove_cmd 서비스 호출"""
        if not self.tilt_move_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('TiltMove_cmd 서비스를 찾을 수 없습니다!')
            return
        
        # 서비스 요청 생성
        request = TiltAngleMove.Request()
        request.target_angle = float(target_angle)
        request.playtime = int(playtime)
        
        # 서비스 호출
        self.get_logger().info(f'TiltMove_cmd 서비스 호출 (angle={target_angle}도, playtime={playtime})')
        future = self.tilt_move_client.call_async(request)
        future.add_done_callback(self.pan_tilt_response_callback)
    
    def pan_tilt_response_callback(self, future):
        """Pan/Tilt 서비스 응답 콜백"""
        try:
            response = future.result()
            if response.command_result:
                self.get_logger().info('Pan/Tilt 명령 성공')
            else:
                self.get_logger().error('Pan/Tilt 명령 실패')
        except Exception as e:
            self.get_logger().error(f'Pan/Tilt 서비스 호출 중 오류: {str(e)}')
    
    def call_conveyor_auto_move(self, start_value):
        """Conveyor_auto_move_cmd 서비스 호출"""
        if not self.conveyor_auto_move_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Conveyor_auto_move_cmd 서비스를 찾을 수 없습니다!')
            return
        
        # 서비스 요청 생성
        request = ConveyorAutoMovement.Request()
        request.start = int(start_value)
        
        # 서비스 호출
        self.get_logger().info(f'Conveyor_auto_move_cmd 서비스 호출 (start={start_value})')
        future = self.conveyor_auto_move_client.call_async(request)
        future.add_done_callback(self.conveyor_response_callback)
    
    def conveyor_response_callback(self, future):
        """Conveyor 서비스 응답 콜백"""
        try:
            response = future.result()
            if response.command_result:
                self.get_logger().info('Conveyor 명령 성공')
            else:
                self.get_logger().error('Conveyor 명령 실패')
        except Exception as e:
            self.get_logger().error(f'Conveyor 서비스 호출 중 오류: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    node = GpioControllerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
