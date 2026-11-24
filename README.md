# tetra_lab

TETRA 로봇의 연구 개발(R&D), 시뮬레이션, 사전 검증 테스트, 외부 통신 브릿지 및 실험적 기능들이 포함된 서브모듈입니다.

---

## 1. 패키지 구성 (Packages Overview)

| 패키지명 | 설명 | 주요 노드 및 런치 |
|---|---|---|
| **`robot_gazebo_sim`** | Gazebo 기반 TETRA 로봇 3D 물리 시뮬레이션 환경 (단일/다중 로봇 스폰, SLAM, Nav2 검증) | `gazebo_sim.launch.py`, `sim_bringup.launch.py`, `sim_navigation.launch.py`, `multi_robot_sim.launch.py` |
| **`tcp_bridge`** | 외부 시스템(PLC, 상위 제어기 등)과 ROS2 간 TCP 소켓 통신을 중계하는 브릿지 패키지 | `tcp_server_node.cpp`, `tcp_client_node.cpp`, `tcp_bridge_server.launch.py`, `tcp_bridge_client.launch.py` |
| **`tetra_gpio_controller`** | TETRA 로봇 GPIO 상태 모니터링 기반 컨베이어, Pan/Tilt 서보, 로봇 팔(Arm) 궤적 제어 노드 | `gpio_controller_node.py`, `gpio_controller.launch.py` |
| **`velocity_integrator`** | 자율주행 속도 토픽(`/cmd_vel_nav`)을 수신하여 속도/이동량 적분 및 연산을 수행하는 유틸리티 노드 | `velocity_integrator_node.cpp`, `velocity_integrator.launch.py` |
| **`tetra_mmwave_radar`** | (실험) DFRobot SEN0395 24GHz mmWave 인체 감지 레이더 UART 드라이버 | `mmwave_radar_node.py`, `mmwave_radar.launch.py` |

---

## 2. 패키지별 상세 가이드 및 실행 방법

### 2.1 Gazebo 3D 시뮬레이션 (`robot_gazebo_sim`)

실제 로봇 없이 Gazebo 가상 환경에서 로봇 스폰 및 센서 데이터 확인, SLAM 매핑, Navigation2 자율주행을 검증합니다.

```bash
# 1. Gazebo 가상 월드 실행 및 로봇 스폰
ros2 launch robot_gazebo_sim gazebo_sim.launch.py

# 2. 시뮬레이션 Bringup (Robot State Publisher + EKF)
ros2 launch robot_gazebo_sim sim_bringup.launch.py

# 3. RViz2 시각화
ros2 launch robot_gazebo_sim rviz_sim.launch.py

# 4. 시뮬레이션 자율주행 (Navigation2)
ros2 launch robot_gazebo_sim sim_navigation.launch.py

# (원클릭) 전체 시뮬레이션 풀 런치
ros2 launch robot_gazebo_sim full_sim.launch.py
```

---

### 2.2 TCP 통신 브릿지 (`tcp_bridge`)

외부 TCP 클라이언트/서버와 ROS2 간 데이터 송수신을 연결합니다.

```bash
# TCP 브릿지 서버 실행
ros2 launch tcp_bridge tcp_bridge_server.launch.py

# TCP 브릿지 클라이언트 실행
ros2 launch tcp_bridge tcp_bridge_client.launch.py
```

---

### 2.3 TETRA GPIO 컨트롤러 (`tetra_gpio_controller`)

TETRA 로봇의 GPIO 상태(`/gpio_status`)를 모니터링하고 입출력 신호 변화에 따라 컨베이어 이동, Pan/Tilt 카메라 구동, 로봇 팔 관절 제어 액션을 수행합니다.

```bash
# Launch 파일로 실행
ros2 launch tetra_gpio_controller gpio_controller.launch.py

# 또는 노드 직접 실행
ros2 run tetra_gpio_controller gpio_controller_node
```

- **구독 토픽**: `/gpio_status` (`interfaces/msg/Gpio`)
- **사용 서비스**:
  - `herkulex/set_angle` (`herkulex_driver/srv/SetAngle`): Pan(서보 1)·Tilt(서보 2) 각도 제어
  - `/conveyor_auto_move_cmd` (`interfaces/srv/ConveyorAutoMovement`): 컨베이어 자동 이동
  - `/power_outport_cmd` (`interfaces/srv/PowerSetOutport`): GPIO 출력 제어
- **사용 액션**:
  - `/joint_trajectory_controller/follow_joint_trajectory` (`control_msgs/action/FollowJointTrajectory`): 로봇 팔 궤적 제어

---

### 2.4 속도 적분기 (`velocity_integrator`)

내비게이션 노드에서 발행되는 속도 명령(`/cmd_vel_nav`)을 수신하여 적분 연산을 수행합니다.

```bash
# Launch 파일로 실행
ros2 launch velocity_integrator velocity_integrator.launch.py

# 또는 노드 직접 실행
ros2 run velocity_integrator velocity_integrator_node
```

- **구독 토픽**: `/cmd_vel_nav` (`geometry_msgs/msg/Twist`)

---

### 2.5 mmWave 인체 감지 레이더 (`tetra_mmwave_radar`, 실험)

DFRobot SEN0395 (24GHz FMCW) 레이더를 USB-UART(115200bps)로 읽어 사람 유무를 발행합니다. 자세한 내용은 [`tetra_mmwave_radar/README.md`](tetra_mmwave_radar/README.md).

```bash
ros2 launch tetra_mmwave_radar mmwave_radar.launch.py port:=/dev/ttyUSB0
ros2 topic echo /mmwave_radar/presence_changed
```
