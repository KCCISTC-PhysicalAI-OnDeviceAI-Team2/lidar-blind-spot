"""융합 노드의 판단 로직(ROS 없이 테스트할 수 있는 함수)."""

from blindspot_fusion import zone


def select_sources(lidar_time, camera_time, stale_sec):
    """
    판단에 쓸 소스(LiDAR, 카메라)를 고른다.

    lidar_time, camera_time: 각 소스의 가장 최근 메시지 시각 [s]. 받은 적 없으면 None.
    stale_sec: 두 소스의 메시지 시각 차이가 이 값[s]보다 크면 더 오래된 쪽을 판단에서 뺀다.

    반환값: (use_lidar, use_camera, notes). notes는 reason에 붙일 문자열 목록.
    """
    use_lidar = lidar_time is not None
    use_camera = camera_time is not None
    notes = []

    if not use_lidar:
        notes.append('lidar none')
    if not use_camera:
        notes.append('camera none')

    if use_lidar and use_camera:
        if lidar_time - camera_time >= stale_sec:
            use_camera = False
            notes.append('camera stale')
        elif camera_time - lidar_time >= stale_sec:
            use_lidar = False
            notes.append('lidar stale')

    return use_lidar, use_camera, notes


def decide(obstacles, stop_x, slow_x, half_width, min_confidence):
    """
    장애물 목록에서 가장 위험한 판단을 고른다.

    obstacles: label, x, y, confidence, source를 가진 장애물 목록
        (blindspot_msgs/Obstacle과 같은 모양). 판단에 쓰는 소스의 장애물만 넣는다.
    min_confidence: 카메라 장애물은 confidence가 이 값 이상일 때만 쓴다.

    반환값: (level, lidar_hit, camera_hit, worst).
    worst는 판단을 정한 장애물이고, 구역 안에 장애물이 없으면 None이다.
    """
    level = zone.PASS
    lidar_hit = False
    camera_hit = False
    worst = None

    for obs in obstacles:
        if obs.source == 'camera' and obs.confidence < min_confidence:
            continue
        obs_level = zone.classify_zone(obs.x, obs.y, stop_x, slow_x, half_width)
        if obs_level == zone.PASS:
            continue

        if obs.source == 'lidar':
            lidar_hit = True
        else:
            camera_hit = True

        if worst is None or obs_level > level or (obs_level == level and obs.x < worst.x):
            level = obs_level
            worst = obs

    return level, lidar_hit, camera_hit, worst
