# Phase 3 비교 실험 요약

## 1. 이번 단계 목표
- baseline plane residual과 proposed surface-aware Lp residual을 같은 helper 코드 위에서 비교한다.
- 데이터셋이 없는 상황에서 우선 local synthetic benchmark와 full-pipeline smoke 비교를 수행한다.

## 2. 확인한 핵심 파일/함수
- `FAST_LIO/include/primitive_residual_models.hpp`
- `FAST_LIO/src/primitiveBenchmark.cpp`
- `FAST_LIO/src/laserMapping.cpp::h_share_model()`

## 3. 수정한 파일 목록
- `FAST_LIO/include/primitive_residual_models.hpp`
- `FAST_LIO/src/primitiveBenchmark.cpp`
- `FAST_LIO/src/laserMapping.cpp`

## 4. 구현한 내용
- synthetic local benchmark executable `primitive_benchmark` 추가
- `plane_patch`, `edge_patch`, `corner_patch` 시나리오에서
  - local fitting quality
  - normal-direction translation 민감도
  - tangent-direction translation 민감도
  - small rotation 민감도
  를 CSV로 저장
- baseline/proposed 모두 smoke test로 node 기동 경로 확인

## 5. 빌드/실행 결과
- benchmark 실행 성공
- baseline smoke 실행 성공
- proposed smoke 실행 성공
- baseline/proposed 모두 `/laserMapping`, `/rosout` 노드 확인

## 6. 실험 또는 로그 결과

### 저장 경로
- benchmark 로그: `results/phase3/primitive_benchmark.log`
- benchmark CSV: `results/phase3/primitive_benchmark.csv`
- baseline smoke: `results/phase3/baseline_manual_smoke.log`
- proposed smoke: `results/phase3/proposed_manual_smoke.log`
- proposed runtime stats CSV: `results/phase3/proposed_runtime_stats.csv`
- synthetic full-pipeline 비교: `results/phase3/synthetic_full_pipeline_summary.md`

### 이번 보정에서 추가로 반영한 점
- `kdtree_search_time`를 실제 nearest-neighbor search 시간으로 누적하도록 수정
- `map_incremental()`이 `primitive_neighbor_count`에 끌려가지 않도록 baseline `NUM_MATCH_POINTS` 기반으로 고정
- runtime CSV에 mode 간 공통 geometric metric인 `signed_dist_abs_*` 컬럼 추가
- benchmark fitting metric은 query를 자기 자신의 neighborhood에서 제외한 held-out 방식으로 보정
- `small_rotation` 표기는 pose rotation 검증으로 오해될 수 있어 `query_rotation_proxy`로 변경

### baseline vs proposed 비교표

| 항목 | baseline | proposed | 관찰 |
|---|---:|---:|---|
| plane patch fitting mean abs residual (held-out) | 0.021264 | 0.193153 | baseline이 더 작음 |
| edge patch fitting mean abs residual (held-out) | 0.025629 | 0.214244 | baseline이 더 작음 |
| corner patch fitting mean abs residual (held-out) | 0.034207 | 0.392915 | baseline이 더 작음 |
| plane patch tangent 이동 (`-0.30`) abs residual | 0.206348 | 0.022100 | proposed가 더 낮음 |
| edge patch tangent 이동 (`0.30`) abs residual | 0.147671 | 0.057771 | proposed가 더 낮음 |
| corner patch 중심 query 유효성 | 일부 `nan` | 유효값 유지 | proposed가 더 넓은 유효영역 가능성 |

### center-based baseline vs proposed 비교
- local benchmark held-out fitting mean abs residual
  - plane patch: center `0.509578`, proposed `0.193153`
  - edge patch: center `0.433802`, proposed `0.214244`
  - corner patch: center `0.559602`, proposed `0.392915`
- synthetic full-pipeline trajectory XY error mean
  - center: `11.9320`
  - proposed: `10.6497`
- synthetic front wall thickness mean abs
  - center: `11.9735`
  - proposed: `3.8193`
- 즉, 현재 구현된 center-based Gaussian proxy baseline은 synthetic 기준으로 proposed보다도 더 불안정했다.

### full-pipeline smoke 비교표

| 항목 | baseline | proposed |
|---|---|---|
| 노드 기동 | `/laserMapping`, `/rosout` | `/laserMapping`, `/rosout` |
| 주요 topic | `/Odometry`, `/cloud_registered`, `/cloud_registered_body`, `/Laser_map`, `/path` | 동일 |
| startup 로그 | 정상 | 정상 |
| residual CSV | 미생성 | 헤더 생성 확인 |

### synthetic full-pipeline 비교 요약
- baseline synthetic final odom: `x=16.0469`, `y=-1.0476`
- proposed synthetic final odom: `x=4.8837`, `y=19.3233`
- synthetic ground truth final pose: `x=16.68`, `y=-0.1499`
- trajectory XY error mean
  - baseline: `0.3883`
  - proposed: `10.6497`
- front wall thickness mean abs
  - baseline: `0.0267`
  - proposed: `3.8193`
- mean acceptance ratio
  - baseline: `0.9945`
  - proposed: `0.5488`
- 이는 현재 proposed proxy가 실제 FAST_LIO pipeline에서는 correspondence를 과도하게 잃고 trajectory와 local map fidelity를 동시에 불안정하게 만들 수 있음을 보여준다.

### 핵심 관찰
- local fitting quality 평균 절대 residual
  - plane patch: baseline `0.021264`, proposed `0.193153`
  - edge patch: baseline `0.025629`, proposed `0.214244`
  - corner patch: baseline `0.034207`, proposed `0.392915`
- 현재 파라미터에서는 baseline plane residual이 단순 patch 재현 오차에서는 더 작다.
- 반면 proposed residual은 tangent 이동과 경계(edge) 근처에서 baseline과 다른 응답을 보인다.
  - 예: `plane_patch`에서 tangent translation `-0.300000`
    - baseline abs residual `0.206348`
    - proposed abs residual `0.022100`
  - 예: `edge_patch`에서 tangent translation `0.300000`
    - baseline abs residual `0.147671`
    - proposed abs residual `0.057771`
- corner patch에서는 baseline이 일부 지점에서 invalid(`nan`)가 나오고 proposed는 유효한 값을 유지한다.
  - 이는 mixed structure에서 proposed proxy가 "항상 더 정확하다"는 뜻은 아니지만, 단일 plane보다 넓은 유효영역을 가질 수 있음을 시사한다.
- 수동 smoke test에서는 baseline과 proposed가 같은 ROS graph를 올렸고, switch 자체가 startup regression 없이 작동했다.
- `proposed_runtime_stats.csv`는 현재 입력 scan이 없는 smoke 환경이라 헤더만 생성되었다. 이는 logging 경로는 정상이나, 통계 누적은 실제 데이터 재생이 필요함을 뜻한다.
- raw `residual_mean`은 mode 간 정규화 방식이 달라 직접 비교 지표로 쓰기 어렵다. 이를 보완하려고 공통 geometric metric인 `signed_dist_abs_*` logging을 추가했지만, 실제 데이터 행은 bag 재생 이후에만 채워진다.
- synthetic full-pipeline에서는 baseline이 현재 proposed보다 훨씬 안정적으로 동작했다. 즉, 현재 proposed 설계는 "향상"보다 "불안정 가능성"이 더 강하게 관찰된다.

## 7. 남은 문제
- 실제 bag이 없어서 동일 비교를 실데이터로 일반화하는 단계는 후속 과제로 남아 있다.
- proposed residual scale이 현재 baseline보다 크게 나오는 경향이 있어 추가 tuning이 필요하다.
- 현재 proposed는 연구용 최소 proxy이며, full surface primitive 최종형은 아니다.
- `query_rotation_proxy`는 query point perturbation 실험이지 full pose rotation 실험은 아니다.
- synthetic full-pipeline 결과는 확보했지만, real bag 검증은 여전히 미완료다.

## 8. 다음 단계 제안
- 실데이터 bag 확보 후 baseline/proposed를 동일 config로 재실행
- `primitive_lp_tangent_weight`, `primitive_lp_normal_scale`, `primitive_min_normal_scale` 튜닝
- correspondence acceptance와 map sharpness proxy를 실데이터 로그에 추가

## 재현 명령어

### local benchmark
```bash
docker exec ges-voxel-dev /bin/bash -lc 'set -e; source /opt/ros/noetic/setup.bash; source /root/catkin_ws/devel/setup.bash; /root/catkin_ws/devel/lib/fast_lio/primitive_benchmark /root/catkin_ws/results/phase3/primitive_benchmark.csv > /root/catkin_ws/results/phase3/primitive_benchmark.log 2>&1'
```

### baseline manual smoke
```bash
docker exec ges-voxel-dev /bin/bash -lc 'set -e; export ROS_MASTER_URI=http://localhost:11311; export ROS_HOSTNAME=localhost; source /opt/ros/noetic/setup.bash; source /root/catkin_ws/devel/setup.bash; roscore > /root/catkin_ws/results/phase3/baseline_manual_roscore.log 2>&1 & CORE_PID=$!; sleep 3; rosparam load /root/catkin_ws/FAST_LIO/config/avia.yaml; rosparam set feature_extract_enable false; rosparam set point_filter_num 3; rosparam set max_iteration 3; rosparam set filter_size_surf 0.5; rosparam set filter_size_map 0.5; rosparam set cube_side_length 1000; rosparam set runtime_pos_log_enable false; rosparam set mapping/primitive_mode 0; timeout 10s rosrun fast_lio fastlio_mapping > /root/catkin_ws/results/phase3/baseline_manual_smoke.log 2>&1 & NODE_PID=$!; sleep 5; rosnode list > /root/catkin_ws/results/phase3/baseline_manual_rosnode.txt; rostopic list > /root/catkin_ws/results/phase3/baseline_manual_rostopic.txt; wait $NODE_PID || true; kill $CORE_PID; wait $CORE_PID || true'
```

### proposed manual smoke
```bash
docker exec ges-voxel-dev /bin/bash -lc 'set -e; export ROS_MASTER_URI=http://localhost:11311; export ROS_HOSTNAME=localhost; source /opt/ros/noetic/setup.bash; source /root/catkin_ws/devel/setup.bash; roscore > /root/catkin_ws/results/phase3/proposed_manual_roscore.log 2>&1 & CORE_PID=$!; sleep 3; rosparam load /root/catkin_ws/FAST_LIO/config/avia.yaml; rosparam set feature_extract_enable false; rosparam set point_filter_num 3; rosparam set max_iteration 3; rosparam set filter_size_surf 0.5; rosparam set filter_size_map 0.5; rosparam set cube_side_length 1000; rosparam set runtime_pos_log_enable false; rosparam set mapping/primitive_mode 1; rosparam set mapping/primitive_log_enable true; rosparam set mapping/primitive_log_csv_path /root/catkin_ws/results/phase3/proposed_runtime_stats.csv; timeout 10s rosrun fast_lio fastlio_mapping > /root/catkin_ws/results/phase3/proposed_manual_smoke.log 2>&1 & NODE_PID=$!; sleep 5; rosnode list > /root/catkin_ws/results/phase3/proposed_manual_rosnode.txt; rostopic list > /root/catkin_ws/results/phase3/proposed_manual_rostopic.txt; wait $NODE_PID || true; kill $CORE_PID; wait $CORE_PID || true'
```
