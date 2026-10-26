# 노드 간 인터페이스

노드끼리 주고받는 토픽, 메시지, 패키지 구성, 좌표, 판단 규칙, 평가 방법, 파일 형식을 정한다. 각 노드가 이 문서의 입출력만 지키면, rosbag을 재생하며 따로 개발한 뒤 그대로 통합할 수 있다.

- 이 문서는 **노드 밖으로 보이는 입력과 출력**만 정한다. 각 노드의 내부 구현은 담당자가 정한다.
- 인터페이스를 바꿀 때는 코드보다 이 문서를 먼저 고친다.
- 담당 역할은 통합 · ROS 2, 데이터 · 평가, 센서 · 캘리브레이션, 모델 · 엣지 AI이며, 아래 표에서는 통합 · 데이터 · 센서 · 모델로 줄여 쓴다.

## 1. 토픽

| 토픽 | 메시지 | 발행 → 구독 |
| --- | --- | --- |
| `/scan` | `sensor_msgs/msg/LaserScan` | LiDAR 드라이버(sllidar_ros2) → 클러스터링 노드 |
| `/camera/image_raw` | `sensor_msgs/msg/Image` | 카메라(`v4l2_camera`, 네임스페이스 `camera`) → 추론 노드, 시각화 노드 |
| `/camera/image_raw/compressed` | `sensor_msgs/msg/CompressedImage` | 카메라(자동 발행) → rosbag 녹화 |
| `/detections` | `vision_msgs/msg/Detection2DArray` | 추론 노드 → 좌표 변환 노드, 시각화 노드 |
| `/detections_overlay` | `sensor_msgs/msg/Image` | 시각화 노드 → 화면 확인 |
| `/camera_obstacles` | `blindspot_msgs/msg/ObstacleArray` | 좌표 변환 노드 → 융합 노드, 평가 스크립트 |
| `/lidar_obstacles` | `blindspot_msgs/msg/ObstacleArray` | 클러스터링 노드 → 융합 노드, 평가 스크립트 |
| `/safety_advice` | `blindspot_msgs/msg/SafetyAdvice` | 융합 · 판단 노드 → 평가 스크립트 |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | bringup launch(고정 좌표 변환) → 좌표 변환이 필요한 노드 |

**QoS:** 센서 토픽(`/scan`, `/camera/image_raw`)을 구독할 때는 `qos_profile_sensor_data`를 쓴다(발행 쪽이 어떤 설정이어도 받을 수 있다). 이 프로젝트의 결과 토픽은 기본 설정(reliable, 깊이 10)으로 발행한다.

## 2. 패키지와 노드

모든 노드는 **Python(rclpy)** 으로 작성한다. 메시지 패키지만 `ament_cmake`로 빌드한다.

| 패키지 · 폴더 | 담당 | 노드 · 내용 |
| --- | --- | --- |
| `blindspot_msgs` | 통합 | 커스텀 메시지 (3장) |
| `blindspot_bringup` | 통합 | 전체 launch, 고정 좌표 변환, 카메라(`v4l2_camera`) 설정 `config/camera.yaml` |
| `blindspot_fusion` | 통합 | `fusion_node` (융합 · 판단) |
| `blindspot_lidar` | 센서 | `lidar_cluster_node` |
| `blindspot_calib` | 센서 | `camera_to_floor_node`, 호모그래피 계산 도구 |
| `blindspot_perception` | 모델 | `detector_node` (TensorRT 추론) |
| `blindspot_viz` | 데이터 | `overlay_node` |
| `eval/` (ROS 패키지 아님) | 통합 | rosbag에서 평가 결과 CSV를 만드는 스크립트 |
| `analysis/` (ROS 패키지 아님) | 데이터 | 결과 그래프, 오검출 분석 스크립트 |
| `training/` (ROS 패키지 아님) | 모델 | 촬영 · 프레임 추출 도구, 학습 · 변환 노트북(출력을 지운 상태). 모델 파일은 올리지 않음 |
| `docs/` | 통합 | 이 문서, 구조도 |

- **담당자는 자기 폴더만 고친다.** 다른 폴더를 고쳐야 하면 담당자에게 요청한다. 이렇게 하면 PR끼리 충돌이 거의 나지 않는다.
- **패키지는 담당자가 첫 코딩 작업에서 만든다.** `ros2 pkg create --build-type ament_python <패키지 이름> --dependencies rclpy <쓰는 메시지 패키지>`. (`blindspot_msgs`만 `ament_cmake`)
- **파라미터는 코드에 직접 쓰지 않고 각 패키지의 `config/<노드 이름>.yaml`에 둔다.** `blindspot_bringup`의 launch가 이 파일들을 불러온다. 파라미터 이름은 담당자가 정한다.
  - 각 패키지는 `setup.py`의 `data_files`에 `('share/' + package_name + '/config', glob('config/*.yaml'))`를 넣는다(넣지 않으면 설치되지 않아 launch가 못 찾는다). launch 파일도 같은 방식으로 `launch/`를 넣는다.
  - YAML은 맨 위를 `/**:` 아래 `ros__parameters:`로 쓴다.
- **Python 라이브러리는 ROS 기본, numpy, OpenCV(cv_bridge)까지로 제한한다.** Jetson에 pip로 패키지를 더 설치하면 NumPy 버전이 바뀌어 `cv_bridge`가 깨질 수 있기 때문이다(NumPy는 1.x로 고정).

### 노드별 입출력

**카메라 — `v4l2_camera` 패키지 사용** (설정 값은 모델 담당, launch는 통합 담당)
- 직접 노드를 만들지 않고 `ros-humble-v4l2-camera`를 쓴다. 압축 이미지 플러그인이 설치돼 있으면 `/camera/image_raw/compressed`도 함께 발행된다.
- **네임스페이스 `camera`로 실행한다**(launch에서 `namespace='camera'`, 명령으로는 `--ros-args -r __ns:=/camera`). 그러면 `/camera/image_raw`, `/camera/image_raw/compressed`, `/camera/camera_info`가 발행된다. 토픽 이름을 바꾸는 remap은 압축 토픽이 따라오지 않을 수 있어 쓰지 않는다.
 설정: `video_device: /dev/video0`, `pixel_format: YUYV`, `image_size: [640, 480]`. 실측 주기 약 13Hz(웹캠의 YUYV 한계, USB 2.0 연결). 웹캠이 지원하는 해상도는 `v4l2-ctl --list-formats-ext`로 확인한다.
- 이 웹캠은 MJPG로 30fps가 나오지만, `v4l2_camera`가 MJPG를 지원하지 않아 쓰지 않는다. 30fps가 필요해지면 `usb_cam`으로 바꾼다(토픽 이름과 메시지는 그대로).

**추론 `detector_node`**
- 입력 `/camera/image_raw` → 출력 `/detections`. 입력은 깊이 1로 구독해 밀린 영상은 버린다(지연이 쌓이지 않게).
- 영상 한 장마다 한 번 발행한다. **검출이 없어도 빈 `Detection2DArray`를 발행한다.**
- `Detection2DArray.header`와 각 `Detection2D.header` = 입력 영상의 `header` 그대로(시각과 프레임).
- `Detection2D` 하나당: `results[0].hypothesis.class_id` = 클래스 이름(문자열), `results[0].hypothesis.score` = 신뢰도, `bbox.center.position.x/y`와 `bbox.size_x/y` = 원본 영상 픽셀 기준 상자.
- 파라미터: `model_path`(Jetson에서는 `~/models/` 아래 파일 경로를 지정하고, 실행 중 자동 다운로드에 의존하지 않는다. 사전학습 모델은 `yolo11n.pt`, `yolo11s.pt`), `conf_threshold`(초기값 0.25). 클래스 이름은 학습 데이터의 라벨 이름과 같게 한다.

**클러스터링 `lidar_cluster_node`**
- 입력 `/scan` → 출력 `/lidar_obstacles`. `/scan` 한 번마다 한 번 발행한다(장애물이 없으면 빈 배열).
- `laser` → `base_jig` 변환은 `/tf_static`에서 tf2로 받는다. 같은 값을 노드 파라미터에 따로 두지 않는다(값은 bringup launch 한 곳에만).
- 점을 `base_jig` 바닥 좌표로 바꾸고, 가까운 점끼리 묶어(묶는 거리, 최소 점 수는 파라미터) 묶음 하나를 장애물 하나로 발행한다. 위치는 묶음 점들의 중심이다.
- `label` = `"unknown"`, `confidence` = 1.0, `source` = `"lidar"`. `header.stamp` = `/scan`의 시각.
- 판단 구역보다 조금 넓은 범위(예: x 0~3m, |y| ≤ 1.5m)의 장애물만 발행한다.

**좌표 변환 `camera_to_floor_node`**
- 입력 `/detections` → 출력 `/camera_obstacles`. `/detections` 한 번마다 한 번 발행한다(빈 배열 포함).
- 상자 **아래 가운데 점**(물체가 바닥에 닿는 점)을 호모그래피로 바닥 좌표(x, y)로 바꾼다. 호모그래피 행렬은 바닥 격자로 구해 `blindspot_calib/config/`에 둔다.
- `label` = class_id, `confidence` = score, `source` = `"camera"`. `header.stamp` = `/detections`의 시각(입력 영상의 시각과 같다).

**시각화 `overlay_node`**
- 입력 `/camera/image_raw`, `/detections` → 출력 `/detections_overlay`(상자와 클래스 이름을 그린 영상). 판단에는 쓰지 않는다.

**융합 · 판단 `fusion_node`** — 규칙은 6장
- 입력 `/lidar_obstacles`, `/camera_obstacles` → 출력 `/safety_advice`.
- 두 입력 중 하나가 들어올 때마다 판단해서 발행한다.

## 3. 커스텀 메시지 패키지 `blindspot_msgs`

모든 노드는 이 패키지를 빌드한 뒤 쓴다. 생성 코드의 인코딩 문제를 피하려고 `.msg` 파일의 주석은 영어로 쓴다.

```text
# Obstacle.msg : one obstacle
string  label        # class name. "unknown" for LiDAR
float32 x            # floor position in base_jig, forward +x [m]
float32 y            # left +y [m]
float32 confidence   # 0-1. 1.0 for LiDAR
string  source       # "lidar" or "camera"

# ObstacleArray.msg
std_msgs/Header header   # stamp = original sensor time, frame_id = "base_jig"
Obstacle[] obstacles     # publish an empty array when nothing is found

# SafetyAdvice.msg
std_msgs/Header header   # stamp = latest original time among the inputs used, frame_id = "base_jig"
uint8 PASS=0
uint8 SLOW=1
uint8 STOP=2
uint8 level
bool  lidar_hit                       # a LiDAR obstacle was inside the zone
bool  camera_hit                      # a camera obstacle was inside the zone
builtin_interfaces/Time lidar_stamp   # stamp of the LiDAR message used (0 if not used)
builtin_interfaces/Time camera_stamp  # stamp of the camera message used (0 if not used)
string reason                         # e.g. "camera:black x=1.02 y=0.01 -> STOP"
```

- **장애물이 없어도 빈 배열을 발행한다.** 그래야 "못 찾음"과 "노드가 멈춤"을 구분하고, 검출률 분모를 셀 수 있다.
- `lidar_stamp`, `camera_stamp`는 평가 때 그 판단에 어떤 입력 메시지가 쓰였는지 찾는 데 쓴다(7장).

## 4. rosbag 녹화와 재생

**평가 · 샘플 rosbag에 반드시 녹화할 토픽:** `/scan`, `/camera/image_raw/compressed`, `/tf_static`

```bash
# 실행 위치: Jetson. 센서는 bringup launch로 켜 둔 상태
mkdir -p ~/bags
ros2 bag record -o ~/bags/<rosbag 이름> /scan /camera/image_raw/compressed /tf_static
```

**재생할 때 주의:** rosbag에는 압축 영상만 들어 있어서, 그대로 재생하면 `/camera/image_raw`를 구독하는 노드(추론, 시각화)는 영상을 받지 못한다. 압축을 풀어 `/camera/image_raw`로 다시 내보내는 노드를 **재생보다 먼저** 켠다. 구독하는 노드도 재생보다 먼저 켠다(`/tf_static`은 재생 시작 때 한 번만 나오므로).

```bash
# 터미널 1: 압축 영상 → /camera/image_raw
ros2 run image_transport republish compressed raw --ros-args \
  -r in/compressed:=/camera/image_raw/compressed -r out:=/camera/image_raw
# 터미널 2: 노드 실행
# 터미널 3: rosbag 재생
ros2 bag play ~/bags/<rosbag 이름>
```

**개발용 샘플 rosbag 두 가지** (용량 때문에 저장소에는 올리지 않는다):
- `sample`: 위 3개 토픽. 지그 고정 상태, 물체 하나를 1.0m · 0°에 두고 10초.
- `sample_det`: `sample`을 재생하며 `detector_node`를 돌려 `/scan`, `/camera/image_raw/compressed`, `/tf_static`, `/detections`를 함께 다시 녹화한 것. YOLO가 없는 개발 PC에서는 이것으로 개발한다.

## 5. 좌표와 단위

| 항목 | 내용 |
| --- | --- |
| 기준 좌표 `base_jig` | **바닥 위**(z = 0이 바닥), LiDAR 회전축 바로 아래 점. 전방 +x, 좌측 +y, 위 +z (ROS 표준 방향) |
| LiDAR 프레임 `laser` | 드라이버 기본값. `base_jig` → `laser`는 지그에서 잰 높이(바닥에서 스캔 평면까지, z)와 방향 차이(yaw)로 정한다. 정면 1.0m에 물체를 두고 `/scan`에서 그 물체가 0° 방향에 찍히는지 확인해 yaw를 기록한다 |
| 카메라 프레임 `camera` | 위치(x, y, z)와 각도는 지그에서 재서 기록한다 |
| 고정 변환 | bringup launch에 `static_transform_publisher`로 넣는다 |
| 단위 | 거리 m, 각도 rad(문서에는 °를 함께 표기), 시간은 ROS 시간 |
| 시각 | 결과 메시지의 `header.stamp`는 원본 센서 데이터의 시각을 그대로 넘긴다 |

## 6. 판단 구역과 융합 규칙

| 구역 | 조건 (`base_jig` 기준) | 판단 |
| --- | --- | --- |
| 정지 구역 | 0 ≤ x < 1.5m, \|y\| ≤ 1.2m 안에 장애물 | STOP |
| 감속 구역 | 1.5 ≤ x < 2.5m, \|y\| ≤ 1.2m 안에 장애물 | SLOW |
| 그 외 | 구역 안에 장애물 없음 (지그 뒤 x < 0 포함) | PASS |

**융합 규칙 (`fusion_node`):**
1. LiDAR와 카메라 각각 **가장 최근 메시지**를 보관한다. 아직 한 번도 받지 않은 쪽은 판단에서 빼고 `reason`에 `camera none`처럼 남긴다.
2. 한쪽 메시지가 다른 쪽보다 0.5초 이상 오래됐으면(`stale_sec`, `header.stamp`끼리 비교) 그쪽은 이번 판단에서 빼고 `reason`에 `camera stale`처럼 남긴다. 컴퓨터 시계가 아니라 메시지 시각끼리 비교하므로 rosbag 재생에서도 똑같이 동작한다.
3. 카메라 장애물은 `confidence`가 기준값(`min_confidence`) 이상인 것만 쓴다.
4. 각 장애물이 어느 구역에 있는지 보고, **가장 위험한 판단**(STOP > SLOW > PASS)을 고른다. LiDAR와 카메라는 OR(둘 중 하나라도 구역 안이면 그 판단).
5. `lidar_hit`, `camera_hit`, 판단에 쓴 메시지의 `lidar_stamp`, `camera_stamp`를 채워 발행한다.

- 구역 크기, `stale_sec`, `min_confidence`는 `blindspot_fusion/config/fusion_node.yaml`에 둔다.
- 평가 스크립트도 `fusion_node`와 같은 구역 판정 함수와 같은 설정 값을 쓴다.

**평가 위치와 기대 판단:** 각도는 LiDAR에서 본 물체의 방향(방위각)이다. 정면 0°, 왼쪽 20°. 경계 근처에 물체를 두면 작은 위치 오차로 판단이 바뀌므로, 평가 위치가 구역 경계에서 0.38m 이상 떨어지게 잡았다(처음 검토한 측면 30°에서는 2.0m 위치가 옆 경계까지 0.20m뿐이어서 20°로 바꿨다).

| 평가 위치 | 정답 위치(x, y) | 가장 가까운 경계까지 | 기대 판단 |
| --- | --- | --- | --- |
| 1.0m · 0° | (1.00, 0.00) | 0.50m (x = 1.5) | STOP |
| 2.0m · 0° | (2.00, 0.00) | 0.50m (x = 1.5, 2.5) | SLOW |
| 1.0m · 20° | (0.94, 0.34) | 0.56m (x = 1.5) | STOP |
| 2.0m · 20° | (1.88, 0.68) | 0.38m (x = 1.5) | SLOW |
| 빈 장면 | 물체 없음 | — | PASS |

- **카메라 화각 확인:** 20° 방향 물체가 웹캠 화면 안에 충분히 들어오는지 평가 녹화 전에 확인한다.
- **녹화 중 사람 위치:** 녹화하는 동안 사람은 지그 뒤(x < 0)에 머문다. 구역 안에 사람이 있으면 빈 장면도 STOP이 된다.

## 7. 평가 방법

검출(물체를 찾았는가)과 판단(정지 · 감속을 냈는가)을 나눠서 잰다. 평가는 rosbag 재생으로 하고, 지연만 Jetson 실시간 실행에서 잰다.

**평가 실행 순서** (Jetson에서 실행하고, 사용한 모델의 이름과 형식을 결과표에 적는다):
1. republish와 노드 전체(카메라 · LiDAR 드라이버 제외)를 켠다.
2. `ros2 bag record -o ~/bags/<rosbag 이름>_out /lidar_obstacles /camera_obstacles /safety_advice`
3. `ros2 bag play ~/bags/<rosbag 이름>`
4. 재생이 끝나면 녹화를 멈춘다(Ctrl+C).
5. `eval/` 스크립트가 `_out` rosbag과 조건표로 평가 결과 CSV를 만든다.

| 지표 | 쓰는 토픽 | 계산 |
| --- | --- | --- |
| LiDAR 단독 검출률 | `/lidar_obstacles` | 정답 위치에서 검출 반경 안에 장애물이 있는 메시지 수 ÷ 전체 메시지 수 |
| 카메라 단독 검출률 | `/camera_obstacles` | 같은 방식 |
| 융합 검출률 | `/safety_advice` + 두 장애물 토픽 | `/safety_advice` 한 건마다, 그 판단에 쓴 LiDAR · 카메라 메시지(`lidar_stamp`, `camera_stamp`로 찾음) 중 하나라도 검출 반경 안에 장애물이 있으면 검출. 검출 건수 ÷ 전체 메시지 수 |
| 판단 정확도 | 융합: `/safety_advice` · 단독: 각 장애물 토픽 | 기대 판단과 같은 메시지 수 ÷ 전체 메시지 수. LiDAR · 카메라 단독의 판단은 스크립트가 같은 구역 판정 함수로 계산한다 |
| 오정지율 | 빈 장면 | STOP인 메시지 수 ÷ 전체 메시지 수 (세 방식 모두) |
| 지연 | 실시간 실행의 결과 토픽 | 받은 시각 − `header.stamp`의 평균과 최대. 같은 Jetson 안에서 잰다 |

- 비율로 비교하므로 토픽마다 발행 주기가 달라도 된다. 메시지 수도 함께 적는다.
- rosbag 재생 중에는 `header.stamp`가 녹화 당시 시각이라 지연 계산에 쓰지 않는다.

### 검출 반경 정하는 법 (초기값 0.2m)

검출 반경은 "정답 위치에서 이만큼 안에 있으면 그 물체를 찾은 것으로 친다"는 기준이다. 너무 작으면 위치 오차 때문에 찾은 물체도 놓친 것으로 세고, 너무 크면 다른 물체를 찾은 것까지 검출로 센다.

1. **평가 본 녹화 전:** 평가에 쓰지 않는 세션을 Jetson에서 녹화한다. 바닥 격자에서 위치를 아는 지점 5곳 이상에 물체를 두고, 재생하며 `/lidar_obstacles`와 `/camera_obstacles`의 위치를 기록한다.
2. **오차 계산:** 위치마다 정답과의 거리(오차)를 구하고, 센서별 최대 오차를 적는다. LiDAR는 물체 앞면을 보므로 물체 크기의 절반 정도 오차가 생길 수 있다.
3. **반경 결정:** 두 센서의 최대 오차 중 큰 값보다 조금 크게(0.05m 단위로 올림), 최소 0.2m · 최대 0.4m 안에서 정한다.
4. **고정:** 평가 rosbag 결과를 보기 **전에** 정하고 이 문서에 적는다. 결과를 본 뒤 반경을 바꾸면 결과에 기준을 맞추는 셈이므로 하지 않는다. 바꿔야 하면 이유를 적고 모든 조건을 다시 계산한다.

## 8. 파일 형식

- **rosbag 이름:** `YYYYMMDD_조건번호_물체_거리_각도`. 예: `YYYYMMDD_c03_black_1m0_20deg`, 빈 장면은 `YYYYMMDD_c00_empty`. 평가 출력은 이름 뒤에 `_out`.
- **조건표(CSV):** `조건번호, 물체, 클래스, 거리_m, 각도_deg, 정답_x_m, 정답_y_m, 조명, 기대_판단, rosbag, 메모`
- **평가 결과(CSV):** `조건번호, 방식(lidar/camera/fusion), 메시지_수, 검출_메시지_수, 검출률, 기대판단_일치_수, STOP_수, 평균_지연_ms, 최대_지연_ms`
- **위치 오차(CSV, 반경 결정용):** `위치번호, 정답_x_m, 정답_y_m, 센서, 측정_x_m, 측정_y_m, 오차_m`
- 모든 CSV는 UTF-8, 첫 줄은 열 이름.
