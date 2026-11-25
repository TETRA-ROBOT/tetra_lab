# tetra_mmwave_radar (실험)

DFRobot **SEN0395** 24GHz mmWave 인체 감지 레이더용 ROS 2 드라이버입니다.
벤더 Arduino 라이브러리([DFRobot_mmWave_Radar](https://github.com/DFRobotdl/DFRobot_mmWave_Radar) V1.0)의
프로토콜을 pyserial로 옮겼습니다. 움직이는 사람뿐 아니라 가만히 있는 사람(호흡 등 미세 움직임)도 감지하며,
거리·방향은 주지 않고 **감지 여부(0/1)만** 출력합니다.

## 배선

USB-UART 어댑터(3.3V 레벨)에 연결합니다. 센서 TX → 어댑터 RX, 센서 RX → 어댑터 TX, 115200 8N1.

## 실행

```bash
ros2 launch tetra_mmwave_radar mmwave_radar.launch.py   # 기본 포트: USB 포트 10.3 (by-path)
ros2 topic echo /mmwave_radar/presence_changed
```

포트 이름은 USB 꽂는 순서에 따라 바뀝니다. 이 PC는 CP2102 두 개의 시리얼 번호가 같아(`0001`)
`/dev/serial/by-id` 가 겹치므로 고정하려면 `/dev/serial/by-path/...` 를 쓰세요.

## 인터페이스 (네임스페이스 `/mmwave_radar`)

| 이름 | 타입 | 설명 |
|---|---|---|
| `presence` | `std_msgs/Bool` | 센서 프레임마다 발행 |
| `presence_changed` | `std_msgs/Bool` | 값이 바뀔 때만 (TRANSIENT_LOCAL, 늦게 구독해도 현재 값 수신) |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | 1Hz. 포트 미개방 ERROR, `stale_timeout_s` 동안 프레임 없음 WARN |
| `apply_config` | `std_srvs/Trigger` | `detection_ranges`, 지연시간 파라미터를 센서 **플래시에 저장** |
| `factory_reset` | `std_srvs/Trigger` | 공장 초기화 |
| `start` / `stop` | `std_srvs/Trigger` | `sensorStart` / `sensorStop` (저장 안 됨) |

## 파라미터 (`config/mmwave_radar.yaml`)

| 이름 | 기본값 | 설명 |
|---|---|---|
| `port`, `baudrate` | `/dev/serial/by-path/...10.3...`, `115200` | 이 PC의 USB 포트 10.3. 다른 PC면 `port:=` 로 지정 |
| `command_terminator` | `none` | 명령 끝 문자. `none`(벤더 라이브러리와 동일) / `crlf` / `lf` |
| `send_start_on_connect` | `true` | 포트를 열 때 `sensorStart` 전송 — `sensorStop` 상태로 남은 센서는 아무것도 보내지 않음 |
| `apply_config_on_start` | `false` | `true` 면 연결 시 아래 설정을 플래시에 기록 |
| `detection_ranges` | `[0.0, 9.0]` | `[시작, 끝, ...]` m, 최대 4구간, 0~9m, 0.15m 단위 |
| `detect_latency_s` / `clear_latency_s` | `0.0` | 감지/해제 보고 지연 (25ms 단위, 최대 1638.375s) |

설정 명령은 `sensorStop → 설정들 → saveCfg → sensorStart` 순서로, 벤더 측정대로 1초 간격으로 보냅니다(서비스 한 번에 4~5초).

## 문제 해결

- **프레임이 안 옴**: 로그에 `sensor: ...` 줄이 하나도 없으면 배선(TX/RX 교차)·전원·보드레이트를 확인하세요.
- **보낸 명령이 그대로 즉시 되돌아오고 `Done` 이 없음**: 어댑터 TX-RX 루프백(센서 미연결)입니다.
- **명령에 `Done` 응답이 없음**: `command_terminator:=crlf` 로 바꿔 보세요. 펌웨어별 차이는 실기에서 미확인입니다.

## 테스트

```bash
cd ros/src/tetra_lab/tetra_mmwave_radar && python3 -m pytest -q test
```

## 상태

- 프로토콜 파서·명령 생성: 단위 테스트 13개
- 노드: 가상 시리얼(socat) 센서 에뮬레이터로 프레임 수신·상태 변화·설정 시퀀스·서비스·diagnostics 확인
- **실제 센서 연결 검증은 아직 안 됨**: 2026-11-24 어댑터 TX-RX 루프백, 2026-11-25 배선 변경 후에도 어댑터 RX 로 수신 0바이트 (모든 보드레이트)
