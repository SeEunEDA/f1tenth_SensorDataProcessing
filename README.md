# f1tenth_SensorDataProcessing

# f1tenth\_SensorDataProcessing

> ROS 2 Foxy · LiDAR **전처리(선택)** + **FTG(Find‑The‑Gap)** 기반 장애물 회피 주행

![ROS2](https://img.shields.io/badge/ROS2-Foxy-22314E?logo=ros\&logoColor=white)
![Python](https://img.shields.io/badge/Python-3.8+-3776AB?logo=python\&logoColor=white)
![License](https://img.shields.io/badge/License-Apache--2.0-green)

본 레포는 과제 2 “센서 데이터 처리”의 결과물로,

* `f1tenth_ftg`: LiDAR 데이터로 빈 공간(gap)을 찾아 조향/속도를 출력하는 핵심 노드
* *(선택)* `sensor_tools`: LiDAR 전처리(클리핑, NaN/Inf 제거, 평활화 등)
  을 포함/연동하는 것을 목표로 합니다. 현재 기본 구성은 `f1tenth_ftg` 단독으로 동작하며, 전처리 노드는 추후 추가 가능합니다.

---

## 📦 Repository Layout

```
.
├─ src/
│  └─ f1tenth_ftg/
│     ├─ f1tenth_ftg/
│     │  ├─ __init__.py
│     │  ├─ ftg_node.py           # FTG 알고리즘 노드(rclpy)
│     │  └─ config/ftg.yaml       # 파라미터 기본값
│     ├─ launch/ftg.launch.py     # FTG 실행 런치
│     ├─ package.xml              # ament_python 패키지 메타
│     ├─ resource/f1tenth_ftg     # ament 마커(빈 파일)
│     ├─ setup.py                 # entry_points/설치 파일
│     └─ setup.cfg                # 실행 스크립트 설치 위치(lib/<pkg>)
├─ config/                         # (선택) 전처리/오버라이드 YAML
├─ README.md
└─ LICENSE
```

---

## 🚀 Quick Start

**사전 준비**

* ROS 2 **Foxy**
* 의존성: `ackermann_msgs`, `sensor_msgs`, `geometry_msgs`, `visualization_msgs`, `std_msgs`, `numpy`
* 시뮬레이터(예: \[`f1tenth_gym_ros`])는 별도 워크스페이스/PC에 설치되어 있어야 합니다.

```bash
# 빌드
cd ~/f1tenth_SensorDataProcessing
colcon build --symlink-install
source /opt/ros/foxy/setup.bash
source install/setup.bash

# (터미널 A) 시뮬 켜기
ros2 launch f1tenth_gym_ros gym_bridge_launch.py

# (터미널 B) FTG 켜기
ros2 launch f1tenth_ftg ftg.launch.py
# LiDAR 토픽이 다르면(예: /ego_racecar/scan):
ros2 param set /ftg_node scan_topic /ego_racecar/scan

# 동작 확인
ros2 topic hz /scan
ros2 topic hz /drive
ros2 topic echo -n 3 /drive
```

> **전처리와 함께 쓰기(선택)**: 전처리 노드가 `/scan_in`→`/scan_out`을 제공한다면, FTG의 `scan_topic`을 `/scan_out`으로 바꾸면 됩니다.

---

## 🔧 Parameters (`ftg.yaml`)

> 런타임에 `ros2 param set /ftg_node <name> <value>` 로 바로 조정 가능. 아래는 대표 항목입니다.

### Topics & Frames

| 이름              | 기본값         | 설명                                                  |
| --------------- | ----------- | --------------------------------------------------- |
| `scan_topic`    | `/scan`     | LiDAR 입력 `sensor_msgs/LaserScan`                    |
| `drive_topic`   | `/drive`    | Ackermann 출력 `ackermann_msgs/AckermannDriveStamped` |
| `cmd_vel_topic` | `/cmd_vel`  | (옵션) Twist 출력 시 토픽                                  |
| `frame_id`      | `base_link` | 차량 기준 프레임                                           |

### Vehicle & Field of View

| 이름                 | 기본값     | 설명                           |
| ------------------ | ------- | ---------------------------- |
| `wheelbase`        | `0.33`  | 차축 간 거리 \[m]                 |
| `max_steering_deg` | `28.0`  | 최대 조향각 제한 \[deg]             |
| `fov_deg`          | `180.0` | 사용 FOV. 좌/우 특정 각도만 사용하고 싶을 때 |

### Preprocessing

| 이름           | 기본값    | 설명                  |
| ------------ | ------ | ------------------- |
| `clip_min`   | `0.02` | 최소 거리 클리핑 \[m]      |
| `clip_max`   | `10.0` | 최대 거리 클리핑 \[m]      |
| `median_k`   | `5`    | 중값 필터 창 크기(0이면 비활성) |
| `mean_k`     | `1`    | 이동 평균 창 크기(0이면 비활성) |
| `downsample` | `2`    | 다운샘플(1=그대로)         |

### Gap Finding & Safety

| 이름                   | 기본값    | 설명                                   |
| -------------------- | ------ | ------------------------------------ |
| `danger_dist`        | `0.50` | 이 이하 근접은 위험으로 간주해 버블로 제거             |
| `bubble_radius`      | `0.30` | 위험점 주변 버블 반경(빔 index 기준 또는 거리 기준 구현) |
| `bestpoint_smooth_k` | `5`    | 갭 내부 최대/평활 포인트 선택 스무딩 강도             |

### Control & Speed

| 이름                 | 기본값    | 설명                          |    |          |
| ------------------ | ------ | --------------------------- | -- | -------- |
| `steer_lpf_alpha`  | `0.6`  | 조향 저역통과(0\~1, 낮을수록 더 부드러움)  |    |          |
| `max_speed`        | `2.5`  | 최대 속도 \[m/s]                |    |          |
| `min_speed`        | `0.5`  | 최소 속도 \[m/s]                |    |          |
| `k_steer_speed`    | `1.0`  |                             | 조향 | 비례 감속 이득 |
| `k_dist_speed`     | `1.5`  | 전방 여유거리 비례 가속 이득(구현에 따라 옵션) |    |          |
| `front_window_deg` | `15.0` | 정면 근방 평균 거리 계산 범위           |    |          |

### Outputs

| 이름                  | 기본값     | 설명                                 |
| ------------------- | ------- | ---------------------------------- |
| `publish_ackermann` | `true`  | `/drive` 직접 발행                     |
| `publish_twist`     | `false` | `/cmd_vel` 발행(과제1 PID 변환과 연동)      |
| `publish_markers`   | `true`  | RViz MarkerArray(`ftg/markers`) 출력 |

---

## 🧪 Tuning Recipes

* **코너에서 소심함** → `danger_dist ↓ (0.35~0.45)`, `bubble_radius ↑ (0.35~0.45)`
* **조향 떨림** → `steer_lpf_alpha ↓ (0.4~0.5)`, `bestpoint_smooth_k ↑ (5~9)`
* **직선 가속 부족** → `max_speed ↑ (3.0~3.5)`, `k_steer_speed ↓ (0.8~1.0)`
* **정면 근접 시 과도 감속** → `k_dist_speed ↓` 또는 `front_window_deg ↑`

실시간 조정 예시:

```bash
ros2 param set /ftg_node danger_dist 0.40
ros2 param set /ftg_node steer_lpf_alpha 0.45
ros2 param set /ftg_node max_speed 3.0
```

---

## 📡 Topics

* **Subscribe**: `/<scan_topic>` (`sensor_msgs/LaserScan`)
* **Publish**: `/<drive_topic>` (`ackermann_msgs/AckermannDriveStamped`)
* **Publish (opt)**: `/<cmd_vel_topic>` (`geometry_msgs/Twist`), `ftg/markers` (`visualization_msgs/MarkerArray`)

---

## 🧰 Troubleshooting

* **런치에서 실행 파일을 못 찾음**: `setup.cfg`가 필요합니다.

  * `setup.cfg`에 실행 스크립트를 `lib/f1tenth_ftg`로 설치하도록 지정되어 있어야 함
* **CMakeLists.txt 에러**: 이 패키지는 `ament_python`. `package.xml`에

  ```xml
  <buildtool_depend>ament_python</buildtool_depend>
  <export><build_type>ament_python</build_type></export>
  ```
* **패키지 인식 실패**: ament 마커/패키지 루트 확인

  * `resource/f1tenth_ftg` **(빈 파일)**, `f1tenth_ftg/__init__.py` **(빈 파일)**
* **환경 문제**: 새 터미널마다

  ```bash
  source /opt/ros/foxy/setup.bash
  source ~/f1tenth_SensorDataProcessing/install/setup.bash
  ```
* **캐시 꼬임**: `rm -rf build install log` 후 재빌드


## 📜 License

Apache-2.0


