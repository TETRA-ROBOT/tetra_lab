# tetra_lab - AI Agent Context

## 개요
R&D 실험, Gazebo 3D 시뮬레이션, 외부 TCP 통신 브릿지, GPIO 기반 부가 장치(컨베이어/팬틸트/로봇 암) 컨트롤러를 관리하는 패키지입니다.

## 서브 패키지 맵

| 서브 패키지 | 언어 | 역할 |
|---|---|---|
| `tetra_gpio_controller` | Python | GPIO 출력 신호(Output 0~7) 변화를 감지하여 컨베이어 구동, Pan/Tilt 서보 각도 이동 및 관절 궤적(Joint Trajectory) 액션 실행 |
| `tcp_bridge` | C++ | 외부 시스템(PLC 등)과 TCP 로 주고받는 브릿지. JSON 이 아니다: 서버는 포트 9000 에서 받은 바이트를 그대로 `from_ros1`(String)로, 클라이언트는 30 ms 마다 `DS,2,1,STATUS,XX\n` 을 보내고 받은 줄을 쉼표로 나눠 토픽으로 낸다 (18칸 미만 줄은 원문만 냄) |
| `robot_gazebo_sim` | Python/XML | Gazebo 물리 시뮬레이션 월드 및 로봇 스폰 런치 |
| `velocity_integrator` | C++ | `/cmd_vel_nav` 의 linear.x 를 적분해 누적 이동 거리 `/distance` 를 낸다 (위치 추정은 아님) |
| `tetra_mmwave_radar` | Python | (실험) DFRobot SEN0395 mmWave 레이더 UART 드라이버. 프로토콜(`protocol.py`, ROS 비의존·단위테스트)과 노드 분리. 설정은 센서 플래시에 저장되므로 기본값은 쓰지 않음 |

## GPIO 제어 매핑

- `output0 == 1`: 컨베이어 시작 (`start=0`)
- `output1 == 1`: 컨베이어 역방향 (`start=8`)
- `output2 == 1`: Pan 90도 회전 (`herkulex/set_angle`, 서보 1. 틸트는 서보 2)
- `output3 == 1`: Pan 0도 복귀
- `output4 == 1`: Tilt 80도 회전 (`herkulex/set_angle`, 서보 2)
- `output5 == 1`: Tilt 0도 복귀
- `output6 == 1`: 관절 궤적(Joint Trajectory) 실행
- `output7 == 1`: 로봇팔 접기 (어깨 80°, 팔꿈치 -150°. 웹 관제의 지금 stow 자세와 다르다)
- MH 브링업 기본은 이 노드를 끈다 (`use_gpio_controller:=false`, 2026-11-24)
