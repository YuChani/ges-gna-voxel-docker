# Phase 3 synthetic full-pipeline 비교

## 1. 이번 단계 목표
- 외부 bag이 없는 상태에서 baseline/proposed를 같은 synthetic sensor stream으로 실제 FAST_LIO pipeline에 통과시킨다.
- smoke test 수준을 넘어서 odometry / runtime / residual 통계를 직접 비교한다.

## 2. 확인한 핵심 파일/함수
- `scripts/synthetic_fastlio_stream.py`
- `scripts/run_synthetic_primitive_compare.sh`
- `FAST_LIO/src/laserMapping.cpp`

## 3. 수정한 파일 목록
- `scripts/synthetic_fastlio_stream.py`
- `scripts/run_synthetic_primitive_compare.sh`

## 4. 구현한 내용
- `MARSIM` 경로를 사용해 `sensor_msgs/PointCloud2 + Imu` synthetic stream 생성
- 3면 환경(정면 wall, 측면 wall, 바닥)을 고정 world로 두고, 가상 센서가 시간에 따라 이동하며 관측하는 scan 생성
- baseline / proposed 모두에 대해
  - `/Odometry` 저장
  - runtime / residual CSV 저장
  - ROS graph 저장
- `run_synthetic_primitive_compare.sh` 한 번으로 `map_quality.json`까지 생성되도록 재현 경로를 일원화

## 5. 빌드/실행 결과
- baseline synthetic full-pipeline 실행 성공
- proposed synthetic full-pipeline 실행 성공
- intensity field를 추가해 `Failed to find match for field 'intensity'` 경고는 제거했다.
- 다만 첫 scan에서 `No point, skip this scan!` 경고 1회는 남아 있었다.

## 6. 실험 또는 로그 결과

### 저장 경로
- baseline runtime: `results/phase3/synth_baseline_cleanintensity/primitive_runtime.csv`
- proposed runtime: `results/phase3/synth_proposed_cleanintensity/primitive_runtime.csv`
- center baseline runtime: `results/phase3/synth_center_repro/primitive_runtime.csv`
- baseline odometry: `results/phase3/synth_baseline_cleanintensity/odometry.csv`
- proposed odometry: `results/phase3/synth_proposed_cleanintensity/odometry.csv`
- center baseline odometry: `results/phase3/synth_center_repro/odometry.csv`
- baseline map-quality proxy: `results/phase3/synth_baseline_cleanintensity/map_quality.json`
- proposed map-quality proxy: `results/phase3/synth_proposed_cleanintensity/map_quality.json`
- center baseline map-quality proxy: `results/phase3/synth_center_repro/map_quality.json`
- one-step reproducibility 확인용 baseline: `results/phase3/synth_baseline_repro/`
- one-step reproducibility 확인용 proposed: `results/phase3/synth_proposed_repro/`

### synthetic ground truth
- 최종 가상 pose: `x=16.68`, `y=-0.1499`

### 비교표

| 항목 | baseline | proposed | 관찰 |
|---|---:|---:|---|
| runtime rows | 134 | 134 | 동일 길이 비교 가능 |
| mean acceptance ratio | 0.9945 | 0.5488 | proposed correspondence가 크게 줄어듦 |
| mean signed_dist_abs_mean | 0.003724 | 0.000335 | 정규화 residual과 별도로 공통 metric 기록 |
| mean match_time (s) | 0.002796 | 0.001748 | proposed가 약간 작음 |
| mean search_time (s) | 0.002633 | 0.003107 | proposed가 약간 큼 |
| mean incremental_time (s) | 0.000237 | 0.000867 | proposed가 더 큼 |
| final odom x | 16.0469 | 4.8837 | baseline이 GT 16.68에 더 가까움 |
| final odom y | -1.0476 | 19.3233 | proposed가 크게 이탈 |
| final xy norm | 16.0810 | 19.9309 | proposed drift 큼 |
| trajectory XY error mean | 0.3883 | 10.6497 | proposed가 크게 악화 |
| front wall thickness mean abs | 0.0267 | 3.8193 | proposed map fidelity 급격히 악화 |
| front wall thickness std | 0.0336 | 2.3839 | proposed wall thickness 크게 증가 |
| side wall thickness mean abs | 0.2945 | 0.4129 | proposed가 더 두꺼움 |

### center-based Gaussian proxy baseline 비교

| 항목 | center baseline | proposed | 관찰 |
|---|---:|---:|---|
| trajectory XY error mean | 11.9320 | 10.6497 | center baseline이 더 나쁨 |
| front wall thickness mean abs | 11.9735 | 3.8193 | center baseline이 훨씬 더 두꺼움 |
| side wall thickness mean abs | 0.5866 | 0.4129 | center baseline이 더 두꺼움 |
| mean acceptance ratio | 0.2409 | 0.5488 | center baseline이 더 적게 채택됨 |

### 핵심 관찰
- synthetic full-pipeline에서는 baseline이 가상 경로를 비교적 안정적으로 따라갔다.
- proposed는 same stream에서 acceptance ratio가 크게 감소했고, trajectory도 크게 이탈했다.
- synthetic map-quality proxy에서도 front wall thickness가 baseline 대비 proposed에서 크게 증가했다.
- center-based Gaussian proxy baseline은 synthetic 기준으로 proposed보다도 더 나빴다.
- 즉, 현재 proposed parameter / residual 설계는 최소 proxy 수준에서는 "개선"보다 "불안정" 쪽 증거가 더 강하다.
- 이 결과는 부정적이더라도 중요한 산출물이다. 현재 구현은 baseline/proposed 차이를 실제 pipeline에서 분리해 관측하는 데는 성공했다.

## 7. 남은 문제
- 이 결과는 synthetic 환경 기준이다.
- 실제 LiDAR/IMU bag에서 같은 경향이 재현되는지는 아직 확인하지 못했다.
- synthetic wall-thickness proxy는 확보했고 one-step runner로 재생성 가능하지만, 실제 LiDAR map sharpness / wall thickness가 동일하게 나타나는지는 아직 미확인이다.

## 8. 다음 단계 제안
- 실데이터 bag 확보 후 같은 자동화 경로를 baseline/proposed에 재사용
- proposed의 `primitive_lp_tangent_weight`, `primitive_min_normal_scale`, acceptance threshold를 재튜닝
- synthetic에서 correspondence collapse를 먼저 줄인 뒤 실데이터로 넘어가기
