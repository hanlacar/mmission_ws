# 교차로 GPS 모범경로 녹화·재현 실차 테스트

```text
WS: ~/mmission_ws

command
0 = NONE
1 = STRAIGHT
2 = RIGHT
3 = LEFT
```

---

# 0. 빌드

## 터미널 1

```bash
cd ~/mmission_ws
source /opt/ros/jazzy/setup.bash

colcon build --symlink-install --packages-select mission_manager

source install/setup.bash
export ROS_DOMAIN_ID=77
```

---

# 1. GPS 실행

## 터미널 2

```bash
cd ~/mmission_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=77

# 현재 사용하는 실제 GPS 노드/launch 실행
ros2 run rtk_gnss rtk_node
```

---

# 2. GPS 확인

## 터미널 3

```bash
source /opt/ros/jazzy/setup.bash
source ~/mmission_ws/install/setup.bash
export ROS_DOMAIN_ID=77

ros2 topic echo /fix
```

주기 확인:

```bash
ros2 topic hz /fix
```

---

# 3. IMU 실행

## 터미널 4

```bash
cd imu_manager

source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=77

# 현재 사용하는 imu_manager 실행 명령
ros2 launch imu_manager imu_manager.launch.py
```

---

# 4. IMU 확인

## 터미널 5

```bash
source /opt/ros/jazzy/setup.bash
source ~/mmission_ws/install/setup.bash
export ROS_DOMAIN_ID=77

ros2 topic echo /imu/relative_yaw_deg
```

다른 터미널에서:

```bash
ros2 topic echo /imu/valid
```

반드시:

```text
/imu/valid = true
```

확인.

---

# 5. N_STRAIGHT 모범경로 녹화

## 터미널 6

````text
N_STRAIGHT 녹화 실행:

```bash
# recorder 실행
ros2 run mission_manager route_recorder --ros-args \
  --params-file src/mission_manager/config/gps_route.yaml \
  -p out_csv:=/home/ww/mmission_ws/src/mission_manager/routes/intersection/N_STRAIGHT.csv \
  -p record_direction:=forward \
  -p record_mode:=INTERSECTION \
  -p record_drive_level:=2.0
````

또는 launch 사용 시:

```bash
ros2 launch mission_manager intersection_record.launch.py route:=N_STRAIGHT
```

```
```

확인:

```bash
ls -lh ~/mmission_ws/src/mission_manager/routes/intersection/N_STRAIGHT.*
```

---

# 6. N_LEFT 모범경로 녹화

## 터미널 6

```bash
ros2 run mission_manager route_recorder --ros-args \
  --params-file src/mission_manager/config/gps_route.yaml \
  -p out_csv:=/home/ww/mmission_ws/src/mission_manager/routes/intersection/N_LEFT.csv \
  -p record_direction:=forward \
  -p record_mode:=INTERSECTION \
  -p record_drive_level:=2.0
```

확인:

```bash
ls -lh ~/mmission_ws/src/mission_manager/routes/intersection/N_LEFT.*
```

---

# 7. N_RIGHT 모범경로 녹화

## 터미널 6

기존 recorder 종료 후 N_RIGHT 녹화 명령 실행.

```bash
ros2 run mission_manager route_recorder --ros-args \
  --params-file src/mission_manager/config/gps_route.yaml \
  -p out_csv:=/home/ww/mmission_ws/src/mission_manager/routes/intersection/N_RIGHT.csv \
  -p record_direction:=forward \
  -p record_mode:=INTERSECTION \
  -p record_drive_level:=2.0
```

확인:

```bash
ls -lh ~/mmission_ws/src/mission_manager/routes/intersection/N_RIGHT.*
```

```전체 확인
ls -lh ~/mmission_ws/src/mission_manager/routes/intersection/
```

---

# 8. 녹화 결과 확인

## 터미널 7

```bash
cd ~/mmission_ws/src/mission_manager/routes/intersection

ls -lh N_STRAIGHT.csv N_LEFT.csv N_RIGHT.csv
```

---

# 9. GPS 경로추종 노드 실행

## 터미널 8

```bash
cd ~/mmission_ws

source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=77

# INTERSECTION_TESTING.md에 작성된
# gps_route_follower 교차로 실행 명령 실행
ros2 launch mission_manager gps_route_follow.launch.py

ros2 launch mission_manager mission.launch.py
```

---

# 10. Publisher 중복 확인

## 터미널 9

```bash
source /opt/ros/jazzy/setup.bash
source ~/mmission_ws/install/setup.bash

ros2 topic info /gps_drive -v
ros2 topic info /gps_wheel -v
```

둘 다 반드시:

```text
Publisher count: 1
```

확인.

---

# 11. 교차로 상태 확인

## 터미널 10

```bash
source /opt/ros/jazzy/setup.bash
source ~/mmission_ws/install/setup.bash

ros2 topic echo /intersection/state
```

---

# 12. CTE / Heading 확인

## 터미널 11

```bash
source /opt/ros/jazzy/setup.bash
source ~/mmission_ws/install/setup.bash

ros2 topic echo /intersection/status
```

주행 중 확인:

```text
cross_track_error_m
heading_error_deg
gps_healthy
imu_healthy
```

---

# 13. 속도 / 조향 확인

## 터미널 12

```bash
source /opt/ros/jazzy/setup.bash
source ~/mmission_ws/install/setup.bash

ros2 topic echo /gps_drive
```

## 터미널 13

```bash
source /opt/ros/jazzy/setup.bash
source ~/mmission_ws/install/setup.bash

ros2 topic echo /gps_wheel
```

정상:

```text
평상시 drive = 2.0
감속     drive = 1.0
정지     drive = 0.0

wheel = -27 ~ +27
```

---

# 14. 완료 신호 확인

## 터미널 14

```bash
source /opt/ros/jazzy/setup.bash
source ~/mmission_ws/install/setup.bash

ros2 topic echo /intersection/complete
```

---

# 15. N방위 선택

## 터미널 15

```bash
source /opt/ros/jazzy/setup.bash
source ~/mmission_ws/install/setup.bash

ros2 param set /gps_route_follower intersection.active_dir N
```

확인:

```bash
ros2 param get /gps_route_follower intersection.active_dir
```

---

# 16. N_STRAIGHT 재현

차량을 `N_STRAIGHT` 녹화 시작 위치에 배치.

## 터미널 16

```bash
ros2 topic pub --once /intersection/command \
std_msgs/msg/UInt8 "{data: 1}"
```

정상 상태:

```text
PREPARE
→ ALIGN_ROUTE
→ TRACKING_STRAIGHT
→ EXIT_CONFIRM
→ COMPLETE
```

완료 시:

```text
/intersection/complete = true
/gps_drive = 0.0
/gps_wheel = 0
```

---

# 17. RESET

## 터미널 16

```bash
ros2 topic pub --once /intersection/command \
std_msgs/msg/UInt8 "{data: 0}"
```

확인:

```text
IDLE
complete=false
```

---

# 18. N_LEFT 재현

차량을 `N_LEFT` 녹화 시작 위치에 배치.

## 터미널 16

```bash
ros2 topic pub --once /intersection/command \
std_msgs/msg/UInt8 "{data: 3}"
```

완료 후:

```text
COMPLETE
complete=true
gps_drive=0.0
gps_wheel=0
```

RESET:

```bash
ros2 topic pub --once /intersection/command \
std_msgs/msg/UInt8 "{data: 0}"
```

---

# 19. N_RIGHT 재현

차량을 `N_RIGHT` 녹화 시작 위치에 배치.

## 터미널 16

```bash
ros2 topic pub --once /intersection/command \
std_msgs/msg/UInt8 "{data: 2}"
```

완료 후:

```text
COMPLETE
complete=true
gps_drive=0.0
gps_wheel=0
```

RESET:

```bash
ros2 topic pub --once /intersection/command \
std_msgs/msg/UInt8 "{data: 0}"
```

---

# 20. 다른 방위 테스트

## E

```bash
ros2 param set /gps_route_follower intersection.active_dir E
```

## S

```bash
ros2 param set /gps_route_follower intersection.active_dir S
```

## W

```bash
ros2 param set /gps_route_follower intersection.active_dir W
```

각 방위에서:

```text
STRAIGHT = 1
RIGHT    = 2
LEFT     = 3
RESET    = 0
```

동일하게 테스트.

---

# 21. 최종 확인

각 경로마다 반드시 확인:

```text
1. 올바른 <방위>_<기동>.csv가 선택됨

2. TRACKING_STRAIGHT / LEFT / RIGHT 진입

3. 평상시 gps_drive = 2.0

4. gps_wheel = -27 ~ +27

5. CTE / heading error 비정상 증가 없음

6. 경로 끝에서 COMPLETE

7. /intersection/complete = true

8. /gps_drive = 0.0

9. /gps_wheel = 0

10. NONE(0) 전까지 정지 유지
```

---

# 실차 시험 순서

```text
GPS 실행
↓
/fix 확인
↓
IMU 실행
↓
IMU valid 확인
↓
모범경로 녹화
↓
gps_route_follower 실행
↓
publisher 1개 확인
↓
active_dir 설정
↓
command 발행
↓
경로 재현
↓
CTE / heading 확인
↓
COMPLETE 확인
↓
0 / 0 정지 확인
↓
NONE reset
```

