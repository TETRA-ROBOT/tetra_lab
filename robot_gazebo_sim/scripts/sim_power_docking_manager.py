#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
sim_power_docking_manager.py

Gazebo 시뮬레이션 환경에서 로봇과 충전스테이션 간의 물리적 접촉/근접 상태를 실시간 감지하여,
실제 하드웨어(tetra_interface)와 동일한 배터리 및 도킹 관련 ROS2 토픽과 서비스를 제공합니다.

발행 토픽:
- /docking_status (std_msgs/msg/Int32): 2 (도킹/충전 중), 1 (도킹 해제/일반 상태)
- /tetra_battery (std_msgs/msg/Int32): 배터리 잔량 (0~100%)
- /battery_voltage (std_msgs/msg/Float64): 배터리 전압 (V)
- /battery_current (std_msgs/msg/Float64): 충/방전 전류 (A)
- /power_status (std_msgs/msg/Int32): 전원 보드 상태 (1)

제공 서비스:
- Power_data_read_cmd (interfaces/srv/PowerDataRead): 실시간 전원 데이터 응답
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32, Float64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Twist
from interfaces.srv import PowerDataRead


class SimPowerDockingManager(Node):
    def __init__(self):
        super().__init__('sim_power_docking_manager')

        # ── 파라미터 선언 ──
        self.declare_parameter('use_sim_time', True)
        self.declare_parameter('odom_topic', 'odom')
        self.declare_parameter('initial_battery', 80.0)             # 초기 배터리 (%)
        self.declare_parameter('charge_rate_pct_per_sec', 0.5)      # 도킹 시 초당 충전율 (%/s)
        self.declare_parameter('discharge_rate_pct_per_sec', 0.02)  # 일반 주행 시 초당 방전율 (%/s)
        self.declare_parameter('dock_dist_thresh', 0.32)            # 도킹 판정 거리 임계값 (m)
        self.declare_parameter('undock_dist_thresh', 0.45)          # 도킹 해제 거리 임계값 (m)
        self.declare_parameter('dock_angle_thresh_deg', 50.0)       # 도킹 대향 각도 허용 오차 (도)

        # 기본 충전스테이션 위치 목록 (tetra_office.sdf 기본 위치 등)
        self.declare_parameter('stations_x', [4.8, 3.0])
        self.declare_parameter('stations_y', [0.0, 0.0])
        self.declare_parameter('stations_yaw', [3.14159, 3.14159])

        # ── 파라미터 로드 ──
        self.odom_topic = self.get_parameter('odom_topic').value
        self.battery_pct = float(self.get_parameter('initial_battery').value)
        self.charge_rate = float(self.get_parameter('charge_rate_pct_per_sec').value)
        self.discharge_rate = float(self.get_parameter('discharge_rate_pct_per_sec').value)
        self.dock_dist_thresh = float(self.get_parameter('dock_dist_thresh').value)
        self.undock_dist_thresh = float(self.get_parameter('undock_dist_thresh').value)
        self.dock_angle_thresh = math.radians(self.get_parameter('dock_angle_thresh_deg').value)

        st_x = self.get_parameter('stations_x').value
        st_y = self.get_parameter('stations_y').value
        st_yaw = self.get_parameter('stations_yaw').value
        self.stations = list(zip(st_x, st_y, st_yaw))

        # ── 상태 변수 ──
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.is_docked = False
        self.last_update_time = self.get_clock().now()
        self.is_moving = False

        # ── Publishers ──
        self.docking_status_pub = self.create_publisher(Int32, 'docking_status', 10)
        self.tetra_battery_pub = self.create_publisher(Int32, 'tetra_battery', 10)
        self.battery_voltage_pub = self.create_publisher(Float64, 'battery_voltage', 10)
        self.battery_current_pub = self.create_publisher(Float64, 'battery_current', 10)
        self.power_status_pub = self.create_publisher(Int32, 'power_status', 10)

        # ── Subscribers ──
        self.odom_sub = self.create_subscription(
            Odometry, self.odom_topic, self._odom_callback, 10)
        
        # 동적 충전스테이션 위치 등록 토픽 (추가 스테이션 스폰 시)
        self.station_pose_sub = self.create_subscription(
            PoseStamped, 'add_charging_station_pose', self._add_station_callback, 10)

        # ── Service ──
        self.power_data_srv = self.create_service(
            PowerDataRead, 'Power_data_read_cmd', self._power_data_callback)

        # ── 10Hz 타이머 (상태 갱신 및 토픽 발행) ──
        self.timer = self.create_timer(0.1, self._update_and_publish)

        self.get_logger().info(
            f'SimPowerDockingManager started | Initial Battery: {self.battery_pct:.1f}% | '
            f'Stations count: {len(self.stations)}')

    def _odom_callback(self, msg: Odometry):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

        # Quaternion to Yaw
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)

        # Speed check
        vx = msg.twist.twist.linear.x
        wz = msg.twist.twist.angular.z
        self.is_moving = (abs(vx) > 0.01 or abs(wz) > 0.01)

    def _add_station_callback(self, msg: PoseStamped):
        sx = msg.pose.position.x
        sy = msg.pose.position.y
        q = msg.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        syaw = math.atan2(siny_cosp, cosy_cosp)
        self.stations.append((sx, sy, syaw))
        self.get_logger().info(f'Added charging station at ({sx:.2f}, {sy:.2f}, yaw={math.degrees(syaw):.1f}°)')

    def _check_docking_contact(self) -> bool:
        """
        로봇의 전면 및 후면 접촉 지점이 임의의 충전스테이션 단자와 접촉했는지 판정합니다.
        """
        rx, ry, ryaw = self.robot_x, self.robot_y, self.robot_yaw

        # 로봇 접촉 후보점 (전면 범퍼/단자 +0.25m, 후면 범퍼/단자 -0.38m)
        front_contact = (rx + 0.25 * math.cos(ryaw), ry + 0.25 * math.sin(ryaw))
        rear_contact = (rx - 0.38 * math.cos(ryaw), ry - 0.38 * math.sin(ryaw))

        thresh = self.undock_dist_thresh if self.is_docked else self.dock_dist_thresh

        for sx, sy, syaw in self.stations:
            # 충전스테이션 전면 단자 위치 (스테이션 중심 기준 +X로 0.10m 돌출)
            station_terminal = (sx + 0.10 * math.cos(syaw), sy + 0.10 * math.sin(syaw))

            # 1) 전면 도킹 검사 (로봇 전면과 스테이션 전면이 마주봄: ryaw ≈ syaw + π)
            d_front = math.hypot(front_contact[0] - station_terminal[0], front_contact[1] - station_terminal[1])
            angle_diff_front = abs(math.atan2(math.sin(ryaw - (syaw + math.pi)), math.cos(ryaw - (syaw + math.pi))))

            if d_front < thresh and angle_diff_front < self.dock_angle_thresh:
                return True

            # 2) 후면 도킹 검사 (로봇 후면과 스테이션 전면이 마주봄: ryaw ≈ syaw)
            d_rear = math.hypot(rear_contact[0] - station_terminal[0], rear_contact[1] - station_terminal[1])
            angle_diff_rear = abs(math.atan2(math.sin(ryaw - syaw), math.cos(ryaw - syaw)))

            if d_rear < thresh and angle_diff_rear < self.dock_angle_thresh:
                return True

        return False

    def _update_and_publish(self):
        now = self.get_clock().now()
        dt = (now - self.last_update_time).nanoseconds / 1e9
        self.last_update_time = now

        if dt <= 0.0 or dt > 1.0:
            dt = 0.1

        # 도킹 접촉 상태 갱신
        docked_now = self._check_docking_contact()
        if docked_now != self.is_docked:
            self.is_docked = docked_now
            state_str = "DOCKING / CHARGING (Status: 2)" if self.is_docked else "UNDOCKED / NORMAL (Status: 1)"
            self.get_logger().info(f'Robot docking state changed: {state_str}')

        # 배터리 잔량 시뮬레이션
        if self.is_docked:
            self.battery_pct = min(100.0, self.battery_pct + self.charge_rate * dt)
            voltage = 24.0 + 4.8 * (self.battery_pct / 100.0)  # 완충 시 약 28.8V
            current = +3.5                                     # 충전 전류 +3.5A
            docking_code = 2                                   # tetra-single-api 도킹 상태 코드 (2: 도킹/충전중)
            charge_signal = 14
            terminal_status = 1
        else:
            discharge = self.discharge_rate * (2.5 if self.is_moving else 1.0)
            self.battery_pct = max(5.0, self.battery_pct - discharge * dt)
            voltage = 23.5 + 4.0 * (self.battery_pct / 100.0)  # 방전 시 23.5 ~ 27.5V
            current = -1.8 if self.is_moving else -0.8        # 방전 전류
            docking_code = 1                                   # tetra-single-api 정상 상태 코드 (1: 미도킹)
            charge_signal = 0
            terminal_status = 0

        # 토픽 발행
        # 1) /docking_status
        msg_dock = Int32()
        msg_dock.data = int(docking_code)
        self.docking_status_pub.publish(msg_dock)

        # 2) /tetra_battery
        msg_bat = Int32()
        msg_bat.data = int(round(self.battery_pct))
        self.tetra_battery_pub.publish(msg_bat)

        # 3) /battery_voltage
        msg_volt = Float64()
        msg_volt.data = float(voltage)
        self.battery_voltage_pub.publish(msg_volt)

        # 4) /battery_current
        msg_curr = Float64()
        msg_curr.data = float(current)
        self.battery_current_pub.publish(msg_curr)

        # 5) /power_status
        msg_pwr = Int32()
        msg_pwr.data = 1
        self.power_status_pub.publish(msg_pwr)

    def _power_data_callback(self, request, response):
        """Power_data_read_cmd 서비스 콜백"""
        voltage_val = int((24.0 + 4.8 * (self.battery_pct / 100.0)) * 10)  # e.g. 288 for 28.8V
        
        response.d0_battery_voltage = voltage_val
        response.d1_system_current = 15
        response.d2_charge_current = 35 if self.is_docked else 0
        response.d3_charge_signal = 14 if self.is_docked else 0
        response.d4_inport_status = 0
        response.d5_outport_status = 0
        response.d6_power_status = 63
        response.d7_charger_terminal_status = 1 if self.is_docked else 0
        response.d8_temperature0 = 310
        response.d9_temperature1 = 285
        response.d10_mobd_inport_status = 15
        response.d11_mobd_outport_status = 0
        response.command_result = True
        return response


def main(args=None):
    rclpy.init(args=args)
    node = SimPowerDockingManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
