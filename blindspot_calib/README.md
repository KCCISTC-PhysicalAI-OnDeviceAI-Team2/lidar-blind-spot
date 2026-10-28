# blindspot_calib

카메라 영상의 픽셀 (u, v)를 바닥 좌표 `base_jig` (x, y) [m]로 바꾸는 호모그래피를 바닥 격자로 구하고, 검출 상자를 바닥 좌표로 바꿔 발행한다.

| 명령 | 하는 일 |
| --- | --- |
| `ros2 launch blindspot_calib camera_to_floor.launch.py` | `camera_to_floor_node` 실행: `/detections` 상자 아래 가운데 점 → `/camera_obstacles` (바닥 좌표) |
| `ros2 run blindspot_calib extract_frame <rosbag>` | 격자 장면 rosbag에서 프레임 한 장을 PNG로, 출처를 JSON으로 저장 |
| `ros2 run blindspot_calib pick_points` | 사진에서 격자 점을 차례로 클릭해 `calib/pixel_points.csv` 저장 |
| `ros2 run blindspot_calib compute_homography` | `config/homography.yaml` 저장(기존 파일은 백업), 점별 오차 출력 · CSV 저장. `--verify`는 저장된 행렬로 오차만 잰다 |
| `ros2 run blindspot_calib measure_points` | 바닥에 표시한 정답 위치(기본: interface 6장 평가 위치)를 변환해 위치별 오차(cm) 출력, interface 8장 위치 오차 CSV · 확인 그림 저장. `--record`면 YAML check 칸에 기록 |

- `extract_frame` 외의 도구는 이 패키지 폴더에서 실행한다(기본 경로가 상대 경로).
- `config/homography.yaml`의 `homography`는 3x3 행렬을 행 우선으로 편 9개 값이다.
  `[x, y, w]^T = H [u, v, 1]^T`, 바닥 좌표는 `(x / w, y / w)`.
- 카메라 높이 · 각도 · 해상도가 바뀌면 다시 캘리브레이션한다.

## camera_to_floor_node

- 입력 `/detections`(`vision_msgs/Detection2DArray`) → 출력 `/camera_obstacles`(`blindspot_msgs/ObstacleArray`, `frame_id` = `base_jig`). 입력 한 번마다 한 번 발행(없으면 빈 배열).
- 상자 아래 가운데 점 `(center_x, center_y + size_y / 2)`을 호모그래피로 바닥 (x, y)로 바꾼다. `label` = class_id, `confidence` = score, `source` = `camera`, `header.stamp` = `/detections` 시각.
- 발행 범위(`config/camera_to_floor_node.yaml`): x 0~3m, 좌우 ±0.7m(interface.md 6장 판단 구역 폭). 범위 밖과 수평선 위 점은 발행하지 않는다.
- 좌표 · 크기 · score가 이상한 검출(nan, 크기 0 이하, score 0~1 밖)과 바닥 접점이 영상 밖인 검출은 빼고 경고한다. 화면 아래 끝에 닿은 상자는 버리지 않고 경고만 한다(잘린 상자면 실제 접점이 아니라 경계 위치).
- 시작할 때 `calibrated` / `validation_performed`가 false이거나 `provisional`이 true면 경고하고, 기록된 check 오차를 로그로 남긴다.
- launch 인자: `params_file`(호모그래피 YAML), `range_params_file`(발행 범위 YAML), `use_sim_time`(bag을 `--clock`으로 재생할 때 true).

## 현재 행렬

- `calib/floor_points.csv` · `calib/pixel_points.csv`의 2026-10-28 줄자 장면 7점으로 계산했다(fit RMS 0.8cm, 하나씩 빼고 잰 추정 오차 RMS 2.8cm · 최대 4.5cm).
- 같은 날 다른 장면 6점으로 잰 check 오차: RMS 3.1cm, 최대 5.9cm(2.5m). `provisional: true`는 팀 확정 전 임시 행렬이라는 뜻이다.

## 오차 기록 (measure_points)

1. 바닥에 `calib/interface_points.csv`의 위치(E1 (1.00, 0), E2 (2.00, 0), E3/E5 (0.94, ±0.34), E4/E6 (1.97, ±0.35))를 줄자로 표시한다. 행렬 계산에 쓴 점과 다른 점이어야 독립 오차다.
2. 그 장면 사진을 찍고 점을 클릭한다.
   `ros2 run blindspot_calib pick_points --image <사진> --floor calib/interface_points.csv --out calib/interface_pixels.csv`
3. 오차를 잰다.
   `ros2 run blindspot_calib measure_points --image <사진>`
   기록까지: `... --record --independent-checks` (3곳 이상 + 사진이면 `validation_performed: true`, 합격 판정 아님)
4. 결과는 `~/calib_work/measure/<날짜_시각>/`의 `position_error_camera_floor.csv`(interface 8장 형식)와 `measure_check.png`.
