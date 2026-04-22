# Compose 기준 real bag 재실행 보고서

## 1. 이번 단계 목표
- `kwu-cvrl/ges-voxel:latest` / `ges-voxel` compose 환경으로 workflow를 다시 맞춘다.
- 같은 환경에서 Livox Avia bag를 baseline / proposed로 재실행한다.

## 2. 확인한 핵심 파일/함수
- `docker-compose/ges-voxel_compose.yml`
- `FAST_LIO/launch/mapping_avia_param.launch`
- `scripts/run_avia_bag_experiment.sh`

## 3. 수정한 파일 목록
- `docker-compose/ges-voxel_compose.yml`
- `FAST_LIO/launch/mapping_avia_param.launch`
- `scripts/run_avia_bag_experiment.sh`

## 4. 구현한 내용
- compose 컨테이너 `ges-voxel`를 재생성해 mount를 아래처럼 바로잡음
  - `/home/chani/personal/dataset -> /root/dataset`
  - repo root -> `/root/catkin_ws`
- Avia bag 실험용 launch override 경로 추가
- compose 컨테이너 안에서 roslaunch + rosbag play 실험 runner 추가

## 5. 빌드/실행 결과
- `ges-voxel` 안에서 `./scripts/bootstrap_fastlio_deps.sh && ./scripts/build_offline.sh` 재실행 성공
- bag: `/root/dataset/sensor_data/livox_avia/2020-09-16-quick-shack.bag`
- baseline 실행 성공
- proposed 실행 성공

## 6. 실험 또는 로그 결과

### 결과 경로
- baseline: `results/phase_real_compose/quick_shack_baseline/`
- proposed: `results/phase_real_compose/quick_shack_proposed/`
- provenance 근거:
  - `docker_ps.txt`
  - `docker_inspect_container.json`
  - `docker_inspect_image.json`
  - `container_runtime.txt`

### baseline vs proposed 요약
- runtime rows
  - baseline: `487`
  - proposed: `487`
- mean acceptance ratio
  - baseline: `0.6894`
  - proposed: `0.2542`
- mean signed_dist_abs_mean
  - baseline: `0.02325`
  - proposed: `0.01394`
- mean search time
  - baseline: `0.001420 s`
  - proposed: `0.001645 s`
- mean incremental time
  - baseline: `0.000066 s`
  - proposed: `0.000379 s`
- final pose
  - baseline: `(-0.0679, -0.0510, 0.0702)`
  - proposed: `(-135.2477, -135.0896, 13.8797)`

### 핵심 관찰
- compose 기준 real bag에서도 proposed는 baseline보다 correspondence acceptance가 크게 낮았다.
- proposed는 최종 pose가 크게 발산해 baseline보다 훨씬 불안정했다.
- 즉, 앞서 synthetic에서 보였던 proposed 불안정성이 real bag 재실행에서도 다시 관찰되었다.

## 7. 남은 문제
- center baseline(`primitive_mode=2`)의 real bag 재실행은 아직 하지 않았다.
- real bag 결과를 기존 최종 보고서와 완전히 통합 정리하는 후속 문서 업데이트는 아직 남아 있다.

## 8. 다음 단계 제안
- 같은 compose 환경에서 center baseline까지 같은 bag로 추가 실행
- real bag 결과를 `results/final_report.md`에 반영
- proposed 파라미터 재튜닝 후 재실행
