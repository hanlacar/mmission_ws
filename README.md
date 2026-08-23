# mission_ws — 대회용 최상위 미션 상태머신

## 무엇인가
GPS/카메라/IMU/엔코더를 통합해 주행을 지휘하는 '두뇌' 워크스페이스.
기존 rtk_node, camera_ws, imu_manager 위에 얹혀 동작한다.

## 구성
- mission_manager_node : 메인 상태머신 (모드 자동전환 + 미션 발동)
- route_recorder_node  : 코스 수동주행하며 waypoint를 CSV로 저장
- gps_health           : 재밍 감지 (timeout/NO_FIX/공분산 + 히스테리시스)
- dead_reckoning       : GPS 끊길 때 엔코더+IMU로 위치 추정
- path_follower        : 저장경로 pure-pursuit 추종
- mission_modes        : T자/평행/가속/우회전 미션 골격
- geo_utils            : 위경도<->로컬미터 변환

## 입력 토픽 (기존 노드가 발행)
- /fix (NavSatFix), /vel (TwistWithCovarianceStamped)   <- rtk_node
- /imu/relative_yaw_deg (Float32)                        <- imu_manager
- /camera/target_speed_mps, /camera/target_steering_deg  <- camera_navigation
- /encoder/ticks (Int32)                                 <- TODO: 엔코더 노드

## 출력 토픽
- /target_speed_mps, /target_steering_deg (Float32)  -> 모터/서보 드라이버(TODO)
- /vehicle_mode (String)  -> imu_manager 등
- /drive_state (String)   -> 디버그/모니터링

## 실행 순서
1) route 저장:  ros2 run mission_manager route_recorder --ros-args -p out_csv:=routes/course.csv
   (코스를 저속 수동주행. 끝나면 origin과 미션존 x,y를 로그에서 확인해 mission.yaml에 기입)
2) 대회 주행:   ros2 launch mission_manager mission.launch.py

## 실차 전 반드시 채울 TODO
- [ ] encoder_m_per_tick : 1m 굴려 tick 세서 mission.yaml에 기입 (추측항법 핵심)
- [ ] /encoder/ticks 실제 토픽명/타입 확인 후 on_encoder 수정
- [ ] 모터/서보 드라이버 노드 (target_speed/steering 구독 -> 실제 구동)
- [ ] mission_modes 각 미션의 실제 궤적 (지금은 골격)
- [ ] 조향 부호(좌+/우-) 실차 확인

---

## 시뮬레이터 (mission_sim) — 하드웨어 없이 검증

실차와 동일한 토픽 구조로 mission_manager를 폐루프 검증한다.
sim 노드가 rtk_node/imu_manager/엔코더 자리를 대신한다.

### 실행
    colcon build
    source install/setup.bash
    ros2 launch mission_sim sim_test.launch.py

- 가상차량이 mission_manager의 명령(/target_speed, /target_steering)으로 움직임
- config/sim.yaml의 jam_zones에서 GPS를 죽여 재밍 재현
- slope_zones에서 경사로 재현
- sim_monitor가 /tmp/sim_result.png로 주행 궤적을 상태색으로 저장

### 검증 완료 (docs/jamming_test.png)
- 재밍 구간을 추측항법으로 통과, 위치오차 평균 0.06m
- GPS 복귀 시 자동으로 GPS 추종 모드로 전환
- 저장경로 완주 성공

### 실차 전환
sim 노드만 빼고 실제 rtk_node / imu_manager / 엔코더노드 / camera_ws로 교체.
mission_manager와 monitor는 그대로 사용.
