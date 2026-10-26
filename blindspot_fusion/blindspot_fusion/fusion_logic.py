"""융합 노드의 판단 로직(ROS 없이 테스트할 수 있는 함수)."""


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
