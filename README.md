# lidar-blind-spot

LiDAR가 놓치는 물체(검은 흡수체, 얇은 물체, 바닥 소형 물체)를 카메라 객체 검출로 찾고, 두 센서를 융합해 통과 / 감속 / 정지 판단(`/safety_advice`)을 내보내는 Jetson Orin Nano 기반 엣지 AI 시스템입니다.

> 진행 중인 팀 프로젝트입니다. 결과는 평가가 끝난 뒤 채웁니다.

- 센서: RPLIDAR C1 (2D LiDAR), USB 웹캠
- 연산: Jetson Orin Nano Super, ROS2 Humble (Ubuntu 22.04, Python 3.10)
- 평가: LiDAR 단독 / 카메라 단독 / 융합의 검출률을 같은 rosbag으로 비교

## 구성

| 폴더 | 내용 | 담당 |
| --- | --- | --- |
| `blindspot_msgs/` | 커스텀 메시지 (`Obstacle`, `ObstacleArray`, `SafetyAdvice`) | A |
| `blindspot_bringup/` | 전체 launch, 고정 좌표 변환, 카메라 설정 | A |
| `blindspot_fusion/` | `fusion_node` (융합 · 판단) | A |
| `blindspot_lidar/` | `lidar_cluster_node` | C |
| `blindspot_calib/` | `camera_to_floor_node`, 호모그래피 도구 | C |
| `blindspot_perception/` | `detector_node` (YOLO TensorRT 추론) | D |
| `blindspot_viz/` | `overlay_node` (검출 시각화) | B |
| `eval/` | rosbag → 평가 결과 CSV | A |
| `analysis/` | 결과 그래프, 오검출 분석 | B |
| `training/` | 프레임 추출, 학습 · 변환 노트북 | D |
| `docs/` | 인터페이스 문서, 구조도 | A |

## 시스템 구조

```text
/scan ──► lidar_cluster_node ──► /lidar_obstacles ───────────────┐
                                                                 ├─► fusion_node ──► /safety_advice
/camera/image_raw ──► detector_node ──► /detections               │
                                          └─► camera_to_floor_node ──► /camera_obstacles ─┘
```

## 환경

- Jetson Orin Nano Super, JetPack 6.2.1, ROS2 Humble, TensorRT 10.3
- 센서: RPLIDAR C1, USB 웹캠 (640×480, 30fps)
- 개발 PC: Ubuntu 22.04 + ROS2 Humble

## 빌드

```bash
mkdir -p ~/blindspot_ws/src && cd ~/blindspot_ws/src
git clone https://github.com/<org>/lidar-blind-spot.git
cd ~/blindspot_ws && colcon build
source install/setup.bash
```

## 결과

(평가 후 작성)

## 라이선스

AGPL-3.0. 이 프로젝트는 [Ultralytics YOLO](https://github.com/ultralytics/ultralytics)(AGPL-3.0)를 사용하므로 같은 라이선스로 배포합니다.
