# FAST_LIO primitive/residual 연구 변경 최종 보고서

## 전체 변경 요약
- baseline build 경로를 Docker 내부에서 재현 가능하게 정리했다.
- `laserMapping.cpp::h_share_model()`에 baseline/proposed primitive-residual 분기 구조를 추가했다.
- `primitive_residual_models.hpp`를 추가해
  - baseline plane residual
  - proposed surface-aware Lp residual proxy
  를 공통 인터페이스로 묶었다.
- primitive 관련 YAML 파라미터를 추가해 런타임 전환이 가능하게 했다.
- residual 통계 CSV logging을 추가했다.
- synthetic local benchmark executable `primitive_benchmark`를 추가했다.
- BALM-style local refinement용 recent-keyframe hook scaffolding을 추가했다.
- Oracle 검토 후, search-time / map-increment coupling / benchmark overclaim 문제를 추가 보정했다.
- 외부 bag이 없을 때도 baseline/proposed를 실제 pipeline에 통과시킬 수 있도록 synthetic full-pipeline 비교 경로를 추가했다.
- synthetic full-pipeline runner는 이제 `map_quality.json`까지 한 번에 생성하는 one-step 재현 경로를 가진다.

## baseline / proposed 차이점

### 비교축에 대한 해석 주의
- 원래 연구 설명에는 Gaussian/center-based baseline 표현이 있었지만, 실제 FAST-LIO backbone의 online matching baseline은 `esti_plane()` 기반 point-to-plane 경로다.
- 그래서 현재 repo는 두 baseline을 모두 가진다.
  - `primitive_mode=0`: FAST-LIO 실제 baseline(point-to-plane)
  - `primitive_mode=2`: center-based Gaussian proxy baseline
- proposed는 `primitive_mode=1`의 surface-aware Lp proxy다.

### baseline
- primitive: local plane
- support: nearest-neighbor subset
- residual: signed point-to-plane distance
- Jacobian direction: plane normal

### center baseline
- primitive: centroid + covariance scale 기반 center Gaussian proxy
- support: nearest-neighbor subset
- residual: ellipsoidal center distance level-set residual
- Jacobian direction: ellipsoid level-set gradient

### proposed
- primitive: local covariance 기반 tangent-normal frame + axis scale + Lp shape parameter
- support: 같은 nearest-neighbor subset
- residual: tangent 위치에 따라 normal consistency를 재가중하는 surface-aware Lp proxy
- Jacobian direction: normal 항 + tangent gate 변화율을 반영한 근사 gradient

## 현재 완료 수준
- 코드 구현 및 Docker build: 완료
- baseline/proposed 전환 스위치와 최소 침습 통합: 완료
- synthetic local benchmark: 완료
- synthetic full-pipeline 비교 및 map-quality proxy 검증: 완료
- BALM-style hook scaffolding과 최소 reevaluation entry: 완료
- 실데이터 일반화 검증: 후속 확장 항목

## 사용한 주요 config
- `mapping/primitive_mode`
  - `0`: baseline plane residual
  - `1`: proposed surface-aware Lp residual
  - `2`: center-based Gaussian proxy baseline
- `mapping/primitive_neighbor_count`
- `mapping/primitive_plane_fit_threshold`
- `mapping/primitive_lp_shape_p`
- `mapping/primitive_lp_tangent_weight`
- `mapping/primitive_min_normal_scale`
- `mapping/primitive_log_enable`
- `mapping/primitive_log_csv_path`
- `mapping/local_refinement_hook_enable`
 - `mapping/local_refinement_translation_step_size`

## 수정 파일 목록
- `FAST_LIO/src/laserMapping.cpp`
- `FAST_LIO/include/primitive_residual_models.hpp`
- `FAST_LIO/include/local_refinement_hook.hpp`
- `FAST_LIO/src/primitiveBenchmark.cpp`
- `FAST_LIO/CMakeLists.txt`
- `FAST_LIO/config/avia.yaml`
- `FAST_LIO/config/mid360.yaml`
- `FAST_LIO/config/velodyne.yaml`
- `FAST_LIO/config/ouster64.yaml`
- `FAST_LIO/config/horizon.yaml`
- `FAST_LIO/config/marsim.yaml`

## 재현 절차
1. Docker 컨테이너에서 baseline dependency bootstrap
2. `./scripts/build_offline.sh` 실행
3. baseline smoke test 실행
4. `primitive_benchmark` 실행
5. manual baseline/proposed smoke 실행

자세한 명령어는 아래 문서를 따른다.
- `results/phase0/analysis_report.md`
- `results/phase1/baseline_report.md`
- `results/phase3/comparison_summary.md`
- `results/phase3/synthetic_full_pipeline_summary.md`
- `results/phase4/local_refinement_hook.md`

## 어떤 가설이 지지되었고 무엇이 아직 불확실한가

### 부분적으로 지지된 점
- proposed residual은 edge/boundary/corner synthetic 시나리오에서 baseline과 다른 민감도 패턴을 만든다.
- corner synthetic patch에서는 baseline이 invalid가 되는 일부 query에서 proposed는 유효 residual을 유지했다.
- 즉, proposed primitive proxy가 mixed local structure에서 plane보다 넓은 유효영역을 줄 가능성은 보였다.
- baseline/proposed 전환 스위치는 실제 ROS node startup 경로에서 모두 정상 동작했다.
- synthetic full-pipeline 경로에서 baseline/proposed의 runtime / correspondence / odometry 차이를 실제로 관측할 수 있게 됐다.
- synthetic full-pipeline 경로에서 trajectory XY error와 wall-thickness proxy까지 직접 비교할 수 있게 됐다.
- center-based Gaussian proxy baseline도 synthetic benchmark / synthetic full-pipeline에서 proposed와 직접 비교했다.

### 아직 불확실한 점
- 현재 synthetic benchmark에서는 절대 fitting error 자체는 baseline plane residual이 더 작다.
- proposed residual scale이 아직 커서, 실제 alignment robustness 향상으로 바로 이어진다고 결론내릴 수 없다.
- smoke 환경에서는 실제 scan 입력이 없어서 residual 통계 CSV가 헤더만 생성되었다. 즉, logging plumbing은 검증됐지만 실데이터 수치는 아직 비어 있다.
- 오히려 현재 synthetic full-pipeline에서는 proposed가 baseline보다 더 불안정하게 나타났다. 따라서 현재 parameter / residual 정의는 추가 수정이 필요하다.
- 즉, 현재 세션의 결론은 "proposed가 synthetic end-to-end 기준에서는 baseline보다 불리하다"는 것이며, 실데이터 일반화는 후속 검증 항목이다.
- 또한 center-based Gaussian proxy baseline은 synthetic 기준에서 proposed보다도 더 불리했다.

## 다음 추천 단계
- 실데이터 bag 확보 후 baseline/proposed를 동일 초기화, 동일 launch 조건으로 반복 실행
- proposed 파라미터 튜닝
  - `primitive_lp_tangent_weight`
  - `primitive_lp_normal_scale`
  - `primitive_min_normal_scale`
- full pipeline 로그에 correspondence acceptance, residual mean/std, wall-thickness proxy를 추가
- 그 다음에만 BALM-style local BA entry를 실제로 활성화
