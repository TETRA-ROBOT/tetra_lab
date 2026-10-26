# tetra_lab - AI Agent Context

## 개요
R&D 실험, Gazebo 3D 시뮬레이션, 외부 TCP 통신 브릿지, GPIO 기반 부가 장치(컨베이어/팬틸트/로봇 암) 컨트롤러를 관리하는 패키지입니다.

## 서브 패키지 맵

| 서브 패키지 | 언어 | 역할 |
|---|---|---|
| `tetra_gpio_controller` | Python | GPIO 출력 신호(Output 0~7) 변화를 감지하여 컨베이어 구동, Pan/Tilt 서보 각도 이동 및 관절 궤적(Joint Trajectory) 액션 실행 |
| `tcp_bridge` | C++ | 외부 시스템(PLC, MES, 외부 관제 서버)과 JSON/TCP 소켓으로 명령 및 텔레메트리를 송수신하는 브릿지 노드 |
| `robot_gazebo_sim` | C++/XML | Gazebo 물리 시뮬레이션 월드 및 로봇 스폰 런치 |
| `velocity_integrator` | C++ | 속도 명령 적분 및 누적 위치 추정 노드 |

## GPIO 제어 매핑

- `output0 == 1`: 컨베이어 시작 (`start=0`)
- `output1 == 1`: 컨베이어 역방향 (`start=8`)
- `output2 == 1`: Pan 90도 회전 (`herkulex/set_angle`, 서보 1. 틸트는 서보 2)
- `output3 == 1`: Pan 0도 복귀
- `output4 == 1`: Tilt 80도 회전 (`herkulex/set_angle`, 서보 2)
- `output5 == 1`: Tilt 0도 복귀
- `output6 == 1`: 관절 궤적(Joint Trajectory) 실행
