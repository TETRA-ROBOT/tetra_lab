# TETRA GPIO Controller

이 패키지는 TETRA 로봇의 GPIO 상태를 모니터링하고 로봇 팔과 Pan/Tilt를 제어하는 ROS2 노드입니다.

## 기능

- `gpio_status` 토픽 구독하여 GPIO 상태 모니터링
- `output0` 변화 감지 시 (0→1):
  - Conveyor_auto_move_cmd로 start=0 전송
  - output0을 0으로 리셋
- `output1` 변화 감지 시 (0→1):
  - Conveyor_auto_move_cmd로 start=8 전송
  - output1을 0으로 리셋
- `output2` 변화 감지 시 (0→1):
  - PanMove_cmd로 90도 회전
  - output2를 0으로 리셋
- `output3` 변화 감지 시 (0→1):
  - PanMove_cmd로 0도 이동
  - output3을 0으로 리셋
- `output4` 변화 감지 시 (0→1):
  - TiltMove_cmd로 80도 이동
  - output4를 0으로 리셋
- `output5` 변화 감지 시 (0→1):
  - TiltMove_cmd로 0도 이동
  - output5를 0으로 리셋
- `output6` 변화 감지 시 (0→1):
  - 모든 조인트를 0도로 이동 (6초)
  - output6을 0으로 리셋
- `output7` 변화 감지 시 (0→1):
  - shoulder 80도, elbow -150도로 이동 (6초)
  - output7을 0으로 리셋

## 빌드

```bash
cd ~/Downloads/test
colcon build --packages-select tetra_gpio_controller
source install/setup.bash
```

## 실행

### 방법 1: launch 파일 사용
```bash
ros2 launch tetra_gpio_controller gpio_controller.launch.py
```

### 방법 2: 직접 실행
```bash
ros2 run tetra_gpio_controller gpio_controller_node
```

## 의존성

- rclpy
- interfaces (커스텀 메시지 및 서비스 - Gpio, PowerSetOutport, ConveyorAutoMovement)
- control_msgs
- std_msgs
- action_msgs
- herkulex (Pan/Tilt 제어)

## 토픽 및 서비스

### 구독하는 토픽
- `/gpio_status` (interfaces/msg/Gpio): GPIO 입출력 상태

### 사용하는 서비스
- `/PanMove_cmd` (herkulex/srv/PanAngleMove): Pan 각도 제어
- `/TiltMove_cmd` (herkulex/srv/TiltAngleMove): Tilt 각도 제어
- `/Conveyor_auto_move_cmd` (interfaces/srv/ConveyorAutoMovement): Conveyor 자동 이동 제어
- `/Power_outport_cmd` (interfaces/srv/PowerSetOutport): GPIO 출력 제어

### 사용하는 액션
- `/joint_trajectory_controller/follow_joint_trajectory` (control_msgs/action/FollowJointTrajectory): 로봇 팔 궤적 제어
