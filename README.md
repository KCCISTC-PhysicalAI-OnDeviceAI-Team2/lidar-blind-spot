# LiDAR Blind Spot

**카메라 · LiDAR 융합으로 LiDAR 사각지대 물체를 찾는 Jetson 엣지 AI**

LiDAR가 놓치는 물체(검은 흡수체, 얇은 물체, 바닥 소형 물체)를 카메라 객체 검출로 찾고, 두 센서를 융합해 통과 / 감속 / 정지 판단(`/safety_advice`)을 내보내는 Jetson Orin Nano 기반 엣지 AI 시스템입니다.

> 진행 중인 팀 프로젝트입니다. 결과는 평가가 끝난 뒤 채웁니다.

- 센서: RPLIDAR C1 (2D LiDAR), USB 웹캠
- 연산: Jetson Orin Nano Super, ROS2 Humble (Ubuntu 22.04, Python 3.10)
- 평가: LiDAR 단독 / 카메라 단독 / 융합의 검출률을 같은 rosbag으로 비교

## 구성

| 폴더 | 내용 | 담당 |
| --- | --- | --- |
| `blindspot_msgs/` | 커스텀 메시지 (`Obstacle`, `ObstacleArray`, `SafetyAdvice`) | 통합 |
| `blindspot_bringup/` | 전체 launch, 고정 좌표 변환, 카메라 설정 | 통합 |
| `blindspot_fusion/` | `fusion_node` (융합 · 판단) | 통합 |
| `blindspot_lidar/` | `lidar_cluster_node` | 센서 |
| `blindspot_calib/` | `camera_to_floor_node`, 호모그래피 도구 | 센서 |
| `blindspot_perception/` | `detector_node` (YOLO TensorRT 추론) | 모델 |
| `blindspot_viz/` | `overlay_node` (검출 시각화) | 데이터 |
| `eval/` | rosbag에서 평가 결과 CSV를 만드는 스크립트 | 통합 |
| `analysis/` | 결과 그래프, 오검출 분석 | 데이터 |
| `training/` | 프레임 추출, 학습 · 변환 노트북 | 모델 |
| `docs/` | [노드 간 인터페이스](docs/interface.md), 구조도 | 통합 |

담당은 통합 · ROS 2, 데이터 · 평가, 센서 · 캘리브레이션, 모델 · 엣지 AI의 네 역할입니다. 노드 사이의 토픽, 메시지, 좌표, 판단 규칙은 [docs/interface.md](docs/interface.md)에 정리했습니다.

## 시스템 구조

```text
/scan                         /camera/image_raw
  │                                   │
  ▼                                   ▼
lidar_cluster_node              detector_node
  │                                   │ /detections
  │                                   ▼
  │                         camera_to_floor_node
  │ /lidar_obstacles                  │ /camera_obstacles
  └─────────────────┬─────────────────┘
                    ▼
               fusion_node
                    │
                    ▼
             /safety_advice
```

## 환경

- Jetson Orin Nano Super, JetPack 6.2.1, ROS2 Humble, TensorRT 10.3
- 센서: RPLIDAR C1, USB 웹캠 (640×480)
- 개발 PC: Ubuntu 22.04 + ROS2 Humble

## 빌드

```bash
mkdir -p ~/blindspot_ws/src && cd ~/blindspot_ws/src
git clone https://github.com/KCCISTC-PhysicalAI-OnDeviceAI-Team2/lidar-blind-spot.git
cd ~/blindspot_ws && colcon build
source install/setup.bash
```

## 결과

(평가 후 작성)

## 라이선스

AGPL-3.0. 이 프로젝트는 [Ultralytics YOLO](https://github.com/ultralytics/ultralytics)(AGPL-3.0)를 사용하므로 같은 라이선스로 배포합니다.
