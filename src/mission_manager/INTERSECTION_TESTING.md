# GPS reference-route 교차로 실차 시험

정상 운용은 `gps_route_follower` 한 노드가 기록 경로를 `NavigationController`와
Pure Pursuit로 추종하며 `/gps_drive`, `/gps_wheel`을 발행한다. 예전의 고정 최대조향
+ IMU 90도 로직은 `reference_route` 모드에서 비활성화되며 자동 fallback도 없다.

> 모든 `NOT VERIFIED` 값은 RTK 횡오차와 실차 기하를 측정하기 전 초기값이다.

## 1. 빌드

```bash
cd /home/ww/mmission_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select mission_manager
source install/setup.bash
```

## 2. 센서 실행과 확인

IMU는 실제 `imu_manager` 워크스페이스를 source한 터미널에서 실행한다.

```bash
ros2 run imu_manager imu_manager_node
ros2 topic info /imu/relative_yaw_deg -v
ros2 topic info /imu/valid -v
ros2 topic hz /imu/relative_yaw_deg
```

RTK 실행 명령은 이 저장소에 포함되지 않은 차량별 `rtk_node` launch를 사용한다.
반드시 다음 계약이 보인 다음 진행한다.

```bash
ros2 topic info /fix -v
ros2 topic info /vel -v
ros2 topic hz /fix
ros2 topic echo /fix --once
```

## 3. 교차로 노드 실행

터미널 A는 authoritative GPS controller, 터미널 B는 latched MCU mode publisher다.

```bash
# 터미널 A
cd /home/ww/mmission_ws && source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 launch mission_manager gps_route_follow.launch.py
```

```bash
# 터미널 B
cd /home/ww/mmission_ws && source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 launch mission_manager mission.launch.py
```

명령 전에는 교차로 상태가 `IDLE`이고 교차로 때문에 출력이 움직이지 않아야 한다.

```bash
ros2 topic echo /intersection/state --once
ros2 topic echo /gps_drive --once
ros2 topic echo /gps_wheel --once
```

## 4. publisher와 토픽 계약 확인

```bash
ros2 topic info /gps_drive -v
ros2 topic info /gps_wheel -v
ros2 topic info /intersection/command -v
ros2 topic info /intersection/complete -v
ros2 topic info /intersection/state -v
ros2 topic info /intersection/status -v
ros2 topic info /mcu/mode_code -v
```

정상 조건에서 `/gps_drive`, `/gps_wheel`의 publisher count는 각각 1이며 노드는
`/gps_route_follower`여야 한다.

## 5. 12개 실제 경로 녹화

차량이 완전히 정지한 상태에서 한 명령만 실행하고, 사람이 충분한 연석 clearance를
두고 안전하게 모범 주행한다. 각 명령은 종료할 때 `Ctrl-C`한다. CSV와 같은 이름의
YAML metadata가 함께 생성된다. 기존 파일을 덮어쓰기 전에 반드시 백업한다.

```bash
ros2 run mission_manager route_recorder --ros-args --params-file src/mission_manager/config/gps_route.yaml -p out_csv:=/home/ww/mmission_ws/src/mission_manager/routes/intersection/N_STRAIGHT.csv -p record_direction:=forward -p record_mode:=INTERSECTION -p record_drive_level:=2.0
ros2 run mission_manager route_recorder --ros-args --params-file src/mission_manager/config/gps_route.yaml -p out_csv:=/home/ww/mmission_ws/src/mission_manager/routes/intersection/N_LEFT.csv -p record_direction:=forward -p record_mode:=INTERSECTION -p record_drive_level:=2.0
ros2 run mission_manager route_recorder --ros-args --params-file src/mission_manager/config/gps_route.yaml -p out_csv:=/home/ww/mmission_ws/src/mission_manager/routes/intersection/N_RIGHT.csv -p record_direction:=forward -p record_mode:=INTERSECTION -p record_drive_level:=2.0
ros2 run mission_manager route_recorder --ros-args --params-file src/mission_manager/config/gps_route.yaml -p out_csv:=/home/ww/mmission_ws/src/mission_manager/routes/intersection/E_STRAIGHT.csv -p record_direction:=forward -p record_mode:=INTERSECTION -p record_drive_level:=2.0
ros2 run mission_manager route_recorder --ros-args --params-file src/mission_manager/config/gps_route.yaml -p out_csv:=/home/ww/mmission_ws/src/mission_manager/routes/intersection/E_LEFT.csv -p record_direction:=forward -p record_mode:=INTERSECTION -p record_drive_level:=2.0
ros2 run mission_manager route_recorder --ros-args --params-file src/mission_manager/config/gps_route.yaml -p out_csv:=/home/ww/mmission_ws/src/mission_manager/routes/intersection/E_RIGHT.csv -p record_direction:=forward -p record_mode:=INTERSECTION -p record_drive_level:=2.0
ros2 run mission_manager route_recorder --ros-args --params-file src/mission_manager/config/gps_route.yaml -p out_csv:=/home/ww/mmission_ws/src/mission_manager/routes/intersection/S_STRAIGHT.csv -p record_direction:=forward -p record_mode:=INTERSECTION -p record_drive_level:=2.0
ros2 run mission_manager route_recorder --ros-args --params-file src/mission_manager/config/gps_route.yaml -p out_csv:=/home/ww/mmission_ws/src/mission_manager/routes/intersection/S_LEFT.csv -p record_direction:=forward -p record_mode:=INTERSECTION -p record_drive_level:=2.0
ros2 run mission_manager route_recorder --ros-args --params-file src/mission_manager/config/gps_route.yaml -p out_csv:=/home/ww/mmission_ws/src/mission_manager/routes/intersection/S_RIGHT.csv -p record_direction:=forward -p record_mode:=INTERSECTION -p record_drive_level:=2.0
ros2 run mission_manager route_recorder --ros-args --params-file src/mission_manager/config/gps_route.yaml -p out_csv:=/home/ww/mmission_ws/src/mission_manager/routes/intersection/W_STRAIGHT.csv -p record_direction:=forward -p record_mode:=INTERSECTION -p record_drive_level:=2.0
ros2 run mission_manager route_recorder --ros-args --params-file src/mission_manager/config/gps_route.yaml -p out_csv:=/home/ww/mmission_ws/src/mission_manager/routes/intersection/W_LEFT.csv -p record_direction:=forward -p record_mode:=INTERSECTION -p record_drive_level:=2.0
ros2 run mission_manager route_recorder --ros-args --params-file src/mission_manager/config/gps_route.yaml -p out_csv:=/home/ww/mmission_ws/src/mission_manager/routes/intersection/W_RIGHT.csv -p record_direction:=forward -p record_mode:=INTERSECTION -p record_drive_level:=2.0
```

실차 경로는 `routes/intersection/`, synthetic 데이터는
`routes/test/intersection/`에만 둔다. 후자는 **TEST ONLY — DO NOT USE ON VEHICLE**다.

## 6. 방위와 명령

노드명과 parameter 이름은 다음과 같다. active command 도중 변경은 거부된다.

```bash
ros2 param set /gps_route_follower intersection.active_dir N
ros2 param set /gps_route_follower intersection.active_dir E
ros2 param set /gps_route_follower intersection.active_dir S
ros2 param set /gps_route_follower intersection.active_dir W
```

```bash
# STRAIGHT
ros2 topic pub --once /intersection/command std_msgs/msg/UInt8 "{data: 1}"
# RIGHT
ros2 topic pub --once /intersection/command std_msgs/msg/UInt8 "{data: 2}"
# LEFT
ros2 topic pub --once /intersection/command std_msgs/msg/UInt8 "{data: 3}"
# COMPLETE/FAULT 해제
ros2 topic pub --once /intersection/command std_msgs/msg/UInt8 "{data: 0}"
```

manual 모드는 `<active_dir>_<command>.csv`를 선택한다. auto 모드는 `/vel`의 최근 ENU
진입 heading을 N/E/S/W로 변환한다. 오늘 기본은 manual이다.

## 7. 주행 중 관찰

```bash
ros2 topic echo /intersection/state
ros2 topic echo /intersection/complete
ros2 topic echo /intersection/status
ros2 topic echo /gps_drive
ros2 topic echo /gps_wheel
ros2 topic echo /mcu/mode_code
```

`/intersection/status` JSON의 `cross_track_error_m`, `heading_error_deg`,
`route_progress`, `gps_healthy`, `imu_healthy`를 기록한다. 정상 drive는 2.0, 큰 조향
또는 CTE/heading warning은 최대 1.0, fault는 drive=0/wheel=0이다. 주행 중 MCU mode는
3, COMPLETE/FAULT는 4다.

goal은 마지막 segment, 마지막점 거리, 최종 tangent heading, GPS/IMU health가 모두
정상인 상태를 연속 5회 확인해야 COMPLETE가 된다. COMPLETE는 `true`와 0/0 출력을
NONE까지 유지하며 자동으로 일반 GPS나 카메라로 넘어가지 않는다.

## 8. fault 시험

- GPS invalid: RTK 입력을 끊거나 NO_FIX를 공급한다. `FAULT`, 0/0, mode 4 확인.
- GPS stale: `/fix`를 중단하고 `intersection` controller의 timeout 이후 동일 확인.
- GPS jump: 5 m 초과 불연속 fix를 공급하고 동일 확인.
- IMU invalid: `/imu/valid=false`를 공급하고 동일 확인.
- IMU stale: yaw 토픽을 중단하고 0.20 s 이후 동일 확인.
- start validation: 경로 첫점에서 1.0 m 밖 또는 heading 30도 밖에서 시작해 정지 확인.
- CTE: 0.20 m 이상에서 감속, 0.35 m 이상 연속 3회에서 FAULT 확인.
- heading: 10도 이상에서 감속, 20도 이상 연속 3회에서 FAULT 확인.

모든 fault는 카메라나 IMU-only turn으로 fallback하지 않는다. 원인을 제거한 뒤 NONE을
발행하고 차량을 올바른 시작 pose에 놓은 다음 새 command를 보낸다.

## 9. 오늘 첫 실차 순서

1. 구동륜을 띄우고 sensor/topic/publisher count를 확인한다.
2. command 없이 교차로 출력이 움직이지 않는지 확인한다.
3. `N_STRAIGHT`를 녹화하고 재현해 정상 drive 2.0을 확인한다.
4. 경로 끝 COMPLETE=true, drive=0, wheel=0을 확인하고 NONE reset한다.
5. `N_LEFT`, `N_RIGHT`를 각각 녹화·재현한다.
6. E/S/W도 같은 순서로 반복한다.
7. 경로 시작점에서 일부러 벗어나 start validation을 확인한다.
8. CTE warn 감속과 CTE stop FAULT를 확인한다.
9. IMU invalid/stale 정지를 확인한다.
10. GPS invalid/stale/jump 정지를 확인한다.

## 10. 미실측 항목

`wheelbase_m`, `max_steering_deg`, `steering_sign`, start radius/heading tolerance,
CTE warn/stop, heading warn/stop, goal distance/heading tolerance, confirmation counts,
GPS covariance/jump, 각 sensor timeout은 모두 실차 로그로 재검증해야 한다. 특히 조향
좌/우 부호와 RTK 정지·곡선 구간 횡오차를 먼저 측정한다.

## 개발자 참고: legacy

`intersection.py`의 IMU ±90도 `IntersectionController`는 이전 테스트 호환을 위해
남아 있지만 `intersection.control_mode: reference_route`에서는 생성되지 않는다.
GPS/IMU fault 시 이 로직으로 자동 전환하지 않는다.
