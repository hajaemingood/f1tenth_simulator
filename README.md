# F1TENTH Simulator 데이터 수집 및 라벨 생성

2025 F1TENTH RoboRacer 대회용 머신러닝 데이터를 만들기 위해 시뮬레이터에서 주행 데이터를 기록하고, `/ferrari/scan`의 각 라이다 포인트를 다음 네 클래스 중 하나로 분류한다.

| CSV 라벨 | 의미 | 라벨 생성 기준 |
| --- | --- | --- |
| `isWall` | 트랙 벽 | 점유 지도 이미지에서 추출한 벽 좌표와 비교 |
| `isStatic` | 정적 장애물 | `isStatic_coord.csv`의 장애물 위치·방향과 박스 크기로 판정 |
| `isOpponent` | 상대 차량 | 같은 라이다 프레임 시각에 기록된 상대 차량 TF 위치와 차체 박스로 판정 |
| `isFree` | 위 세 클래스에 속하지 않는 측정 포인트 | 유효 라이다 포인트 중 나머지 |

> `isFree`는 라이다 빔이 지나간 전체 공간의 음성 라벨이 아니라, 유효한 라이다 측정점이 벽·장애물·상대 차량 박스에 해당하지 않는다는 뜻이다. 클래스가 겹칠 때는 후보 위치와의 거리가 가장 가까운 클래스를 남기므로, 출력의 네 라벨은 한 포인트에서 동시에 참이 되지 않는다.

## 1. 환경 준비 및 빌드

저장소에는 ROS 2 Foxy와 Gazebo가 설치된 Docker 환경 설정이 포함되어 있다. 호스트에서 컨테이너를 시작한다.

```bash
cd ~/f1tenth_simulator/docker_f1tenth
xhost +local:docker
docker compose up -d --build
docker exec -it f1_simulator bash
```

컨테이너 안에서 작업공간을 빌드하고 ROS 환경을 불러온다.

```bash
cd /root/f1tenth_simulator
source /opt/ros/foxy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

각 `ros2 launch` 명령은 별도의 컨테이너 터미널에서 실행한다. 새 터미널을 열 때마다 `source /opt/ros/foxy/setup.bash`와 `source /root/f1tenth_simulator/install/setup.bash`를 실행한다.

## 2. 시뮬레이터 실행 순서

### 터미널 1: 두 차량과 Gazebo 실행

```bash
ros2 launch ferrari dual_ferrari.launch.py
```

이 런치는 `ferrari`와 `ferrari_op` 두 차량을 Gazebo에 생성한다. 현재 런치 파일은 `intersection_with_static.world`를 직접 지정하고 있으므로 기본 장면은 교차로와 정적 장애물이다.

### 터미널 2: 두 차량의 오도메트리 실행

```bash
ros2 launch odometry dual_wheel_odom.launch.py
```

두 차량 각각의 wheel odometry와 EKF 노드를 실행한다. TF 트리를 구성하는 데 필요하므로 차량을 움직이거나 bag을 기록하기 전에 실행한다.

### 터미널 3: 두 차량의 지도 좌표 위치 추정 실행

```bash
ros2 launch path_planner dual_amcl.launch.py
```

이 런치는 기본으로 `intersection.yaml` 지도를 사용하고 두 차량의 `map → odom → base_link` TF를 방송한다. 상대 차량 포즈와 라이다 점을 동일한 `map` 좌표에서 비교할 수 있도록 데이터 수집 전에 실행한다.

### 차량 주행

수집 대상 라이다는 주 차량인 `/ferrari/scan`이다. 주 차량을 움직이는 제어 노드와 상대 차량을 움직이는 제어 노드를 필요한 방식으로 실행한다. 예를 들어 키보드 텔레옵은 다음 런치를 사용한다.

주 차량을 키보드로 조종할 때:

```bash
ros2 launch ferrari key_teleop_ferrari.launch.py
```

상대 차량을 키보드로 조종할 때:

```bash
ros2 launch ferrari key_teleop_ferrari_op.launch.py
```

상대 차량은 다음 웨이포인트 주행으로 움직일 수도 있다.

```bash
ros2 launch path_planner pure_pursuit.launch.py
```

키보드 텔레옵과 웨이포인트 주행은 모두 `ferrari_op`를 제어하므로 상대 차량에는 둘 중 하나만 실행한다. 두 차량을 각각 키보드로 조종한다면 별도의 입력 장치/환경을 사용한다. 수집 중에는 상대 차량이 라이다 시야에 들어오고 나가는 다양한 상황을 포함하도록 주행한다.

## 3. ROS 2 bag 기록

시뮬레이터, 오도메트리, AMCL이 실행되고 TF가 게시되는 것을 확인한 뒤 별도 터미널에서 기록을 시작한다.

```bash
cd /root/f1tenth_simulator
mkdir -p src/machine_learning/bagfiles
ros2 bag record -o src/machine_learning/bagfiles/intersection_run_01 \
  /ferrari/scan /tf /tf_static
```

위 세 토픽은 라벨 생성에 필요한 최소 토픽이다.

- `/ferrari/scan`: 분류할 거리 측정값과 프레임 시각
- `/tf`, `/tf_static`: 각 스캔 시각의 차량·센서 TF 및 프레임 연결 정보

주행을 마치면 bag 기록 터미널에서 `Ctrl+C`를 눌러 정상 종료한다. 후처리기는 이 디렉터리의 `metadata.yaml`과 `.db3` 파일을 읽는다. 생성된 bag은 용량이 크며 Git에서 제외되므로 별도 저장소에 백업한다.

기록된 토픽과 메시지 수는 다음으로 확인한다.

```bash
ros2 bag info src/machine_learning/bagfiles/intersection_run_01
```

최소한 `/ferrari/scan`, `/tf`, `/tf_static`이 포함되어야 한다. 주행 제어 명령이나 wheel odometry까지 분석·재생할 필요가 있으면 해당 토픽도 `ros2 bag record` 명령에 추가한다. 기록 중 TF가 빠지면 좌표 변환 또는 상대 차량 라벨이 누락될 수 있다.

## 4. 지도 및 장애물 좌표 준비

후처리는 bag의 센서 데이터와 장면의 정답 좌표 데이터를 대조한다. 현재 저장소에는 교차로 기준 좌표 파일이 있다.

- 벽: `src/machine_learning/config/map_coord.csv`
- 정적 장애물: `src/machine_learning/config/isStatic_coord.csv`

벽 좌표를 다시 만들 때는 맵 YAML/PGM 설정을 확인한 다음 추출기를 실행한다.

```bash
python3 src/machine_learning/machine_learning/map_coord_extractor.py
```

추출기는 `src/ferrari/maps/intersection.yaml`이 가리키는 이미지의 어두운 픽셀(기본 임계값 100 미만)을 벽으로 취급하고, 이미지 바깥 테두리도 벽 좌표에 추가해 `map_coord.csv`에 저장한다. 다른 트랙을 수집한다면 스크립트의 `yaml_path`와 출력 경로를 해당 맵에 맞게 변경한다.

정적 장애물 CSV에는 장애물 중심의 `x`, `y`, `yaw`가 필요하다. 현재 파일에는 Gazebo world 좌표와 방향이 저장되어 있으며, 분류기는 기본으로 `0.5 m × 0.4 m` 박스와 `0.1 m` 허용 오차를 사용한다. 다른 장면이나 맵으로 바꿀 때는 장애물 좌표와 world/map 좌표 변환값이 실제 장면과 일치하는지 확인한다.

## 5. 상대 차량 좌표 추출 및 학습 CSV 생성

후처리 스크립트는 `ros2 run`으로 등록되어 있지 않으므로 ROS 환경을 source한 컨테이너 안에서 Python 스크립트로 직접 실행한다. **상대 차량 좌표를 먼저 추출하고, 그 결과를 분류기에 전달한다.**

```bash
cd /root/f1tenth_simulator
BAG=src/machine_learning/bagfiles/intersection_run_01
OPPONENT_CSV=src/machine_learning/outputs/intersection_run_01_ferrari_op.csv
TRAINING_CSV=src/machine_learning/outputs/intersection_run_01.csv

# 1) 각 /ferrari/scan 프레임 시각에 대응하는 bar_op의 map 좌표와 yaw 추출
python3 src/machine_learning/machine_learning/bar_op_coord.py \
  "$BAG" -o "$OPPONENT_CSV"

# 2) LaserScan 점을 map 좌표로 바꾸고 네 클래스 라벨을 생성
python3 src/machine_learning/machine_learning/rosbag_to_csv.py \
  "$BAG" -o "$TRAINING_CSV" \
  --opponent-csv "$OPPONENT_CSV" \
  --wall-map src/machine_learning/config/map_coord.csv \
  --static-obstacles src/machine_learning/config/isStatic_coord.csv
```

첫 번째 스크립트는 `/ferrari/scan`의 프레임 순서를 기준으로 TF에서 `bar_op`의 `global_x`, `global_y`, `yaw`를 추출한다. 두 번째 스크립트는 각 유효 라이다 측정의 거리와 각도로 센서 좌표를 만들고 `base_link`, `map` 좌표로 변환한다. 같은 프레임의 상대 차량 박스, 벽 좌표, 정적 장애물 박스에 점이 들어가는지 검사해 가장 가까운 후보를 라벨로 기록한다.

학습용 CSV의 주요 열은 다음과 같다.

| 열 | 설명 |
| --- | --- |
| `frame_index`, `stamp_sec`, `stamp_nsec` | 스캔 프레임 순서와 메시지 시각 |
| `scan_index`, `distance` | LaserScan 배열 인덱스와 측정 거리 |
| `local_x`, `local_y` | `base_link` 기준 점 좌표 |
| `global_x`, `global_y` | `map` 기준 점 좌표 |
| `isOpponent`, `isWall`, `isStatic`, `isFree` | 포인트별 boolean 분류 라벨 |

생성된 파일은 `src/machine_learning/outputs/`에서 확인한다. 프레임에 대응하는 상대 차량 TF가 bag에 없으면 상대 차량 위치를 찾을 수 없다는 경고가 출력되고 그 프레임의 `isOpponent`은 모두 `False`가 될 수 있으므로, 경고와 결과 CSV를 반드시 확인한다.

차량 자세를 별도 분석할 필요가 있으면 다음 추출기를 추가로 실행할 수 있다. 이 CSV는 라이다 포인트 학습용 CSV와 별개의 차량 궤적 결과다.

```bash
python3 src/machine_learning/machine_learning/odom_extractor.py \
  "$BAG" -o src/machine_learning/outputs/intersection_run_01_odom.csv
```

## 6. 다른 트랙 데이터 수집 시 주의

`dual_ferrari.launch.py`의 Gazebo 월드 경로와 `dual_amcl.launch.py`의 맵 경로는 현재 각각 교차로 월드와 `intersection.yaml`로 설정되어 있다. 다른 트랙을 수집할 때는 월드, AMCL 맵, `map_coord.csv`, `isStatic_coord.csv`, world-to-map 변환값이 모두 같은 트랙을 가리키도록 맞춘다. 기본 런치에 트랙 선택 인자는 없으므로 다른 트랙으로 전환한 뒤에는 빌드/설치된 런치가 변경 사항을 반영하는지 확인한다.

분류기의 기본 좌표 변환 및 허용 오차는 `rosbag_to_csv.py`의 인자로 변경할 수 있다. 측정 환경과 맞지 않는 값을 임의로 유지하면 특히 벽과 정적 장애물 라벨이 어긋날 수 있다.
