# LiDAR Blind Spot

**2D LiDAR가 놓치는 낮은 · 반투명 장애물을 카메라로 찾아, 통과 · 감속 · 정지를 판단하는 Jetson 엣지 AI**

2D LiDAR는 바닥과 나란한 한 평면만 스캔합니다. 스캔 높이보다 낮은 물체는 레이저가 위로 지나가고, 반투명한 물체는 점이 잘 찍히지 않습니다. 이 프로젝트는 위에서 비스듬히 내려다보는 카메라의 YOLO 검출을 LiDAR와 같은 바닥 좌표로 합치고, 앞 구역에 장애물이 있는지에 따라 통과 · 감속 · 정지(`/safety_advice`)를 Jetson Orin Nano에서 실시간으로 판단합니다.

**결과** (17개 조건, 같은 녹화로 LiDAR만 · 카메라만 · 융합 비교)

- LiDAR만으로 0%였던 낮은 물체(눕힌 상자 · 멀티탭)를 융합 후 **97.5~100%** 검출
- 물체가 없는 장면에서 잘못 멈춤 **0회** (판단 455번 모두 통과)
- Jetson Orin Nano 실시간 판단 지연 평균 **51ms** (최대 110ms)

**환경**

- 센서: RPLIDAR C1 (2D LiDAR, 스캔 높이 12.7cm), Logitech C270 웹캠 (높이 40cm, 아래 17°)
- 연산: Jetson Orin Nano Super, JetPack 6.2.1, ROS 2 Humble, YOLO11n TensorRT FP16
- 실제 로봇 대신 두 센서를 단 지그로 실험했고, 판단 출력까지 다룹니다(모터 제어는 하지 않음).

## 시스템 구조

```text
/scan                         /camera/image_raw
  │                                   │
  ▼                                   ▼
lidar_cluster_node              detector_node (YOLO TensorRT)
  │                                   │ /detections
  │                                   ▼
  │                         camera_to_floor_node (호모그래피)
  │ /lidar_obstacles                  │ /camera_obstacles
  └─────────────────┬─────────────────┘
                    ▼        모두 같은 바닥 좌표 base_jig [m]
               fusion_node
                    │
                    ▼
             /safety_advice  (PASS · SLOW · STOP)
```

- 두 센서의 장애물은 같은 메시지(`blindspot_msgs/Obstacle`: label · x · y · confidence · source)로, LiDAR 바로 아래 바닥을 원점으로 하는 같은 좌표(앞 +x, 왼쪽 +y, 미터)로 보냅니다.
- 토픽, 메시지, 좌표, 판단 규칙, 평가 방법은 [docs/interface.md](docs/interface.md)에 정리했습니다.

## 판단 규칙 (`blindspot_fusion`)

| 항목 | 값 |
| --- | --- |
| 정지 구역 | 앞 0~1.5m, 좌우 ±0.7m |
| 감속 구역 | 앞 1.5~2.5m, 좌우 ±0.7m |
| 합치는 방식 | 두 센서 중 하나라도 구역 안에서 보면 그 판단 |
| 여러 장애물 | 가장 위험한 판단(정지 > 감속 > 통과), 같으면 더 가까운 것 |
| 오래된 입력 | 두 센서의 메시지 시각 차가 0.5초 이상이면 오래된 쪽을 뺌 |
| 카메라 신뢰도 | 0.5 이상만 사용 (평가 녹화 전에 고정) |

- 한쪽이 놓쳐도 다른 쪽이 잡으면 되도록 둘 중 하나만 봐도 판단합니다. 대신 잘못 멈추는 경우가 늘 수 있어 빈 장면의 오정지를 함께 쟀습니다.
- 구역 폭은 처음 ±1.2m였는데, 실험실 양옆 벽(약 0.82m)이 구역에 들어와 물체가 없어도 정지가 나와 ±0.7m로 줄였습니다.
- 오래된 입력을 빼는 것은 한쪽 센서가 늦어졌을 때 지난 장면으로 판단하지 않기 위해서입니다.

값은 `blindspot_fusion/config/fusion_node.yaml`에 있고, 판단 로직(`zone.py`, `fusion_logic.py`)은 ROS 없이 단위 테스트로 확인합니다.

## 결과

### 검출률 (LiDAR만 / 카메라만 / 융합)

조건마다 13초 rosbag을 한 번 재생해 LiDAR 장애물, 카메라 장애물, 융합 판단을 함께 기록하고, 정답 위치(물체에서 지그 쪽으로 가장 가까운 바닥 접점)에서 0.20m 안에 장애물이 있으면 검출로 셌습니다. 정답 위치와 반경은 결과를 보기 전에 정했습니다. 조건은 물체 4종 × 위치 4곳(1m · 2m 정면, 1m 20° · 2m 10° 측면) + 빈 장면 = 17개입니다.

| 물체 | LiDAR가 놓치는 이유 | LiDAR만 | 카메라만 | 융합 |
| --- | --- | ---: | ---: | ---: |
| 눕힌 상자 | 스캔 높이보다 낮음 | 0% | 100% | 100% |
| 멀티탭 | 스캔 높이보다 낮음 | 0% | 97.5% | 97.5% |
| 반투명 컵 | 점이 잘 안 찍힘 | 77% (1m 100, 2m 57) | 100% | 100% |
| 세운 상자 (대조군) | 놓치지 않음 | 100% | 100% | 100% |

- 빈 장면: 융합 판단 455번 모두 통과(정지 0).
- 같은 rosbag을 다른 팀원이 따로 재생 · 평가한 결과와 비교하면 융합 검출률의 조건별 차이는 최대 0.009였습니다.
- 이번 세 물체에서는 카메라만으로도 융합과 같은 수준이었습니다. LiDAR가 카메라를 보완하는 상황(학습하지 않은 물체, 어두운 곳, 카메라 시야 밖)은 이번 평가 범위 밖입니다.

### 실시간 지연 (Jetson Orin Nano, MAXN_SUPER, `ros2 topic delay`, 30초 × 3회)

| 토픽 | 평균 | 최대 |
| --- | ---: | ---: |
| `/lidar_obstacles` | 106ms | 110ms |
| `/camera_obstacles` (YOLO 추론 포함) | 43~44ms | 67ms |
| `/safety_advice` (판단) | 51ms | 110ms |

판단은 초당 약 39.5번 나옵니다. 판단 지연은 판단에 쓴 가장 최근 센서 데이터의 시각부터 잽니다.

### 한계

- 학습한 물체 3종류(`box`, `power_strip`, `clear_cup`), 한 실험실 바닥에서만 확인했습니다.
- 멀티탭 2m 정면은 약 91%로 가장 낮았습니다(원인 미확인).
- rosbag 재생 직후 카메라 결과가 늦게 들어와 융합 판단이 잠깐 LiDAR만으로 나는 경우가 가끔 있습니다(원인 미확인).

## 실행 방법

### 준비

- Ubuntu 22.04 + ROS 2 Humble (Jetson은 JetPack 6.2.1)
- ROS 패키지: `sllidar_ros2`(LiDAR), `ros-humble-v4l2-camera`, `ros-humble-vision-msgs`, `ros-humble-image-transport-plugins`
- Python 패키지를 pip로 설치할 때는 `"numpy<2" "opencv-python<4.12"`를 함께 붙입니다. NumPy 2가 설치되면 Humble의 `cv_bridge`가 동작하지 않습니다.
- 추론 노드는 Jetson에서 만든 TensorRT 엔진을 씁니다(`blindspot_perception/config/detector_node.yaml`의 `model_path`). 모델 파일은 저장소에 넣지 않았습니다. 학습과 변환은 [training/](training/)을 참고하세요.

### 빌드

```bash
mkdir -p ~/blindspot_ws/src && cd ~/blindspot_ws/src
git clone https://github.com/KCCISTC-PhysicalAI-OnDeviceAI-Team2/lidar-blind-spot.git
cd ~/blindspot_ws && colcon build
source install/setup.bash
```

### 실시간 실행 (Jetson)

```bash
# 터미널 1: LiDAR, 카메라, 고정 좌표 변환
ros2 launch blindspot_bringup sensors.launch.py
# 터미널 2: 처리 노드 전체 (overlay:=true면 검출 상자를 그린 영상도 발행)
ros2 launch blindspot_bringup processing.launch.py
# 터미널 3: 판단 확인
ros2 topic echo /safety_advice
```

### rosbag 녹화와 재생

녹화는 센서를 켠 상태에서 `/scan`, `/camera/image_raw/compressed`, `/tf_static`을 저장합니다. rosbag은 용량 때문에 저장소에 넣지 않았습니다.

```bash
# Jetson, 저장소 폴더에서: ~/bags/<이름>에 13초 녹화
bash blindspot_bringup/scripts/record_bag.sh <이름> 13
```

재생할 때는 압축 영상을 `/camera/image_raw`로 다시 내보내는 노드와 처리 노드를 **재생보다 먼저** 켭니다(`/tf_static`이 재생 시작 때 한 번만 나오기 때문).

```bash
# 터미널 1: 압축 영상 → /camera/image_raw
ros2 run image_transport republish compressed raw --ros-args \
  -r in/compressed:=/camera/image_raw/compressed -r out:=/camera/image_raw
# 터미널 2: 처리 노드 (/detections가 이미 든 rosbag이면 detector:=false)
ros2 launch blindspot_bringup processing.launch.py
# 터미널 3: 재생
ros2 bag play ~/bags/<이름>
```

TensorRT가 없는 개발 PC(VM)에서는 Jetson에서 `/detections`까지 함께 녹화한 rosbag을 `detector:=false`로 재생해 나머지 노드를 개발했습니다.

### 평가

재생 중 `/lidar_obstacles`, `/camera_obstacles`, `/safety_advice`를 `<이름>_out`으로 녹화한 뒤, 조건표와 함께 평가 스크립트를 돌립니다. 순서와 지표 정의는 [docs/interface.md 7장](docs/interface.md)에 있습니다.

```bash
python3 eval/evaluate.py --conditions <조건표.csv> --bags ~/bags \
  --config blindspot_fusion/config/fusion_node.yaml --out ~/bags/results.csv
```

### 테스트

`source install/setup.bash` 한 터미널에서 실행합니다.

```bash
cd ~/blindspot_ws
colcon test --packages-select blindspot_fusion && colcon test-result --verbose
cd src/lidar-blind-spot && python3 -m pytest eval/test
```

## 폴더 구성

| 폴더 | 내용 | 담당 |
| --- | --- | --- |
| `blindspot_msgs/` | 메시지 (`Obstacle`, `ObstacleArray`, `SafetyAdvice`) | 통합 |
| `blindspot_bringup/` | 센서 · 처리 launch, 지그 실측 고정 좌표 변환, 카메라 설정, 녹화 스크립트 | 통합 |
| `blindspot_fusion/` | `fusion_node` (융합 · 판단)와 단위 테스트 | 통합 |
| `eval/` | 재생 결과 rosbag으로 방식별 검출률 · 판단 일치율 CSV를 만드는 스크립트 | 통합 |
| `blindspot_lidar/` | `lidar_cluster_node` (점 묶기 → 장애물) | 센서 |
| `blindspot_calib/` | `camera_to_floor_node`, 호모그래피 캘리브레이션 도구 | 센서 |
| `blindspot_perception/` | `detector_node` (YOLO TensorRT 추론) | 모델 |
| `training/` | 학습 데이터 촬영 · 학습 · 변환 | 모델 |
| `blindspot_viz/` | `overlay_node` (검출 상자 시각화) | 데이터 |
| `analysis/` | 결과 그래프 | 데이터 |
| `docs/` | [노드 간 인터페이스](docs/interface.md) | 통합 |

담당은 통합 · ROS 2, 데이터 · 평가, 센서 · 캘리브레이션, 모델 · 엣지 AI의 네 역할입니다.

## 개발 방식

- Jetson은 한 대라, 센서 데이터를 rosbag으로 녹화해 각자 PC에서 같은 데이터로 동시에 개발했습니다. 이 녹화 → 재생 방식은 평가와 시연 영상 제작에도 그대로 썼습니다.
- 노드를 만들기 전에 메시지 형식, 토픽 이름, 좌표, 단위를 [docs/interface.md](docs/interface.md)로 먼저 정했고, 바꿀 때는 문서부터 고쳤습니다.
- 브랜치 → PR → 다른 사람 한 명 승인 → 합치기를 기본 규칙으로 했습니다.

## 라이선스

AGPL-3.0. 이 프로젝트는 [Ultralytics YOLO](https://github.com/ultralytics/ultralytics)(AGPL-3.0)를 사용하므로 같은 라이선스로 배포합니다.
