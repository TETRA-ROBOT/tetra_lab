# tetra_lab

TETRA 로봇의 연구 개발, 사전 검증 테스트, 시뮬레이션 및 실험적 기능들이 포함된 서브모듈입니다.

---

## 1. 패키지 구성

- **`virtual_wall`**: 가상 벽(Virtual Wall)을 생성하여 로봇의 주행 구역을 논리적으로 제한하거나 관리하는 패키지
- **`robot_gazebo_sim`**: 실제 로봇 없이 Gazebo 상에서 TETRA 로봇을 스폰하고 2D LiDAR / Odometry / Navigation2 / SLAM 알고리즘을 테스트하는 3D 물리 시뮬레이션 환경 패키지

---

## 2. Gazebo 시뮬레이션 실행 가이드 (`robot_gazebo_sim`)

### 2.1 Gazebo 시뮬레이션 기본 실행 (월드 + 로봇 스폰)
```bash
ros2 launch robot_gazebo_sim gazebo_sim.launch.py
```

### 2.2 시뮬레이션 Bringup (State Publisher + EKF)
```bash
ros2 launch robot_gazebo_sim sim_bringup.launch.py
```

### 2.3 시뮬레이션 RViz 시각화
```bash
ros2 launch robot_gazebo_sim rviz_sim.launch.py
```

### 2.4 시뮬레이션 환경 내비게이션2 (자율주행 테스트)
```bash
ros2 launch robot_gazebo_sim sim_navigation.launch.py
```

### 2.5 원클릭 전체 시뮬레이션 실행 (Full Simulation)
```bash
ros2 launch robot_gazebo_sim full_sim.launch.py
```

---

## 3. 가상벽 실행 가이드 (`virtual_wall`)

각 로봇 모델별 가상벽 노드 실행:
```bash
ros2 launch virtual_wall virtual_wall_m.launch.py     # M 모델
ros2 launch virtual_wall virtual_wall_h.launch.py     # H 모델
ros2 launch virtual_wall virtual_wall_edu.launch.py   # EDU 모델
ros2 launch virtual_wall virtual_wall_mh.launch.py    # MH 모델
ros2 launch virtual_wall virtual_wall_s.launch.py     # S 모델
ros2 launch virtual_wall virtual_wall_sm.launch.py    # SM 모델
```
