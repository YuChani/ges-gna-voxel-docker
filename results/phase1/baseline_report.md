# Phase 1 baseline 보고서

## 1. 이번 단계 목표
- Docker 내부에서 현재 FAST_LIO baseline을 clean build한다.
- 데이터셋이 없는 상태에서도 최소 실행 절차를 재현한다.
- 이후 baseline/proposed 비교의 기준 로그와 명령어를 고정한다.

## 2. 확인한 핵심 파일/함수
- `scripts/bootstrap_fastlio_deps.sh`: `livox_ros_driver`, `ikd-Tree`, `src/FAST_LIO` 링크 준비
- `scripts/build_offline.sh`: `/root/catkin_ws`에서 `catkin_make -DCMAKE_BUILD_TYPE=Release`
- `FAST_LIO/launch/mapping_avia.launch`: baseline launch 진입점
- `FAST_LIO/config/avia.yaml`: baseline 파라미터 파일

## 3. 수정한 파일 목록
- 없음

## 4. 구현한 내용
- baseline 빌드 경로 확인
- Docker 내부 workspace 경로 확인: `/root/catkin_ws/FAST_LIO`
- baseline smoke test 로그 저장

## 5. 빌드/실행 결과

### baseline build
- 첫 시도 실패 원인: `build_offline.sh`가 `set -u` 상태에서 `ROS_MASTER_URI` 미정의 환경을 가정함
- 조치: `ROS_MASTER_URI`, `ROS_HOSTNAME`를 명시하고 재실행
- 두 번째 실패 원인: `livox_ros_driver` 누락
- 조치: `./scripts/bootstrap_fastlio_deps.sh` 실행 후 재빌드
- 최종 결과: `fastlio_mapping`, `livox_ros_driver_node` 빌드 성공

### baseline smoke test
- 로그: `results/phase1/baseline_smoke.log`
- rosnode 목록: `results/phase1/baseline_rosnode.txt`
- rostopic 목록: `results/phase1/baseline_rostopic.txt`
- 확인 결과:
  - `/laserMapping` 노드 기동
  - `/Odometry`, `/cloud_registered`, `/cloud_registered_body`, `/Laser_map` 등 topic 노출

## 6. 실험 또는 로그 결과
- 현재 `/root/datasets`가 비어 있어서 실제 bag 기반 trajectory/map 비교는 수행하지 못함
- 대신 smoke test 기준선은 확보함

## 7. 남은 문제
- 실측 데이터셋 부재
- baseline residual 통계는 입력 scan이 들어와야 누적 가능

## 8. 다음 단계 제안
- `h_share_model()` 내부에 baseline/proposed 분기 추가
- synthetic local benchmark로 primitive 차이를 먼저 검증

## 재현 명령어

### baseline build
```bash
docker exec ges-voxel-dev /bin/bash -lc 'set -e; export ROS_MASTER_URI=${ROS_MASTER_URI:-http://localhost:11311}; export ROS_HOSTNAME=${ROS_HOSTNAME:-localhost}; cd /root/catkin_ws && ./scripts/bootstrap_fastlio_deps.sh && ./scripts/build_offline.sh'
```

### baseline smoke test
```bash
docker exec ges-voxel-dev /bin/bash -lc 'set -e; export ROS_MASTER_URI=${ROS_MASTER_URI:-http://localhost:11311}; export ROS_HOSTNAME=${ROS_HOSTNAME:-localhost}; source /opt/ros/noetic/setup.bash; source /root/catkin_ws/devel/setup.bash; mkdir -p /root/catkin_ws/results/phase1; timeout 12s roslaunch fast_lio mapping_avia.launch rviz:=false > /root/catkin_ws/results/phase1/baseline_smoke.log 2>&1 & LAUNCH_PID=$!; sleep 6; rosnode list > /root/catkin_ws/results/phase1/baseline_rosnode.txt; rostopic list > /root/catkin_ws/results/phase1/baseline_rostopic.txt; wait $LAUNCH_PID || true'
```
