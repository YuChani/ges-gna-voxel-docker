# Codex 세션 인수인계 문서 (한글)

## 1. 현재 저장소 상태
- 저장소: `YuChani/ges-gna-voxel-docker`
- 현재 실제 수정 대상(active code): `FAST_LIO/`
- 비교 실험용 frozen baseline: `prev/` (**절대 수정 금지**)
- 현재 작업 목적: 연구 최종 포지셔닝이 아니라 **FAST-LIO 측정 경로 붕괴를 막는 엔지니어링 안정화 단계**

## 2. 실행 환경
이 저장소는 host ROS 환경이 아니라 **기존 Docker Compose 환경**에서 작업/실험한다.

- compose 파일: `docker-compose/ges-voxel_compose.yml`
- 서비스/컨테이너 이름: `ges-voxel`
- 컨테이너 내부 workspace: `/root/catkin_ws`
- 컨테이너 내부 dataset 경로: `/root/dataset`

가능하면 기존 스크립트를 재사용한다.
- `./scripts/bootstrap_fastlio_deps.sh`
- `./scripts/build_offline.sh`
- `./scripts/run_avia_bag_experiment.sh`

## 3. 지금까지의 핵심 결론
이전 proposed 방식이 실패한 핵심 원인은, **정규화된 surface residual과 tangent-aware gradient가 FAST-LIO의 기존 point-to-plane measurement path를 직접 대체했기 때문**이다.

즉 문제는 아이디어 자체의 즉시 폐기라기보다,
- correspondence acceptance
- EKF residual
- EKF Jacobian direction
에 surface-aware 항을 너무 직접적으로 넣어 **integration mismatch**가 발생한 것이다.

## 4. 현재 적용된 수정 방향 (Hybrid 구조)
현재는 direct replacement 대신 **hybrid integration**으로 수정되었다.

핵심 원칙:
- correspondence gate: plane signed distance 기반
- EKF residual: 기존 plane residual 유지
- EKF Jacobian direction: plane normal 유지
- surface-aware 정보: confidence weighting / support filtering / logging / refinement metadata 에만 사용

즉,
- plane = 메인 update
- GES/GND-inspired surface-aware 정보 = 보조 정보
구조로 바꾼 상태다.

## 5. 최근 Codex 실행 결과 요약
### Hybrid run 결과
- runtime rows: `487`
- mean acceptance: `0.485036`
- raw signed distance mean/std/max: `0.016211 / 0.017979 / 0.380591`
- surface score mean/std: `0.925325 / 0.070182`
- final pose: `(-0.039334, -0.026460, 0.055136)`
- `No Effective Points!` count: `0`

### 이전 proposed failure
- mean acceptance: `0.254161`
- final pose: `(-135.247693, -135.089559, 13.879704)`
- `No Effective Points!` count: `4`

### 해석
Hybrid mode는 적어도 테스트한 real bag에서는
- acceptance를 회복했고
- catastrophic divergence를 멈췄고
- `No Effective Points!`도 사라졌다.

다만 여전히 baseline acceptance보다는 낮고, 더 많은 bag에서 검증이 필요하다.

## 6. 최근 변경 파일
최근 Codex 보고 기준 변경된 핵심 파일:
- `FAST_LIO/include/primitive_residual_models.hpp`
- `FAST_LIO/src/laserMapping.cpp`
- `FAST_LIO/config/avia.yaml`
- `FAST_LIO/launch/mapping_avia_param.launch`
- `FAST_LIO/include/local_refinement_hook.hpp`
- `scripts/run_avia_bag_experiment.sh`
- 결과 파일: `results/phase_real_hybrid/` 아래

## 7. 최신 git 정보
- branch: `main`
- commit message: `FAST-LIO 하이브리드 GES residual 통합 및 도커 실행 경로 정리`
- commit hash: `9405e865f602ff898c5c50e3e6bfd6cf9f66bd60`

## 8. 기본 실행 방식
추가 launch argument 없이 아래처럼 실행되는 것이 목표이자 현재 기준이다.

- 터미널 1: `roslaunch fast_lio mapping_avia_param.launch`
- 터미널 2: `rosbag play <dataset>.bag`

즉,
- `primitive_mode:=...`
- `rviz:=0`
같은 추가 인자를 기본 proposed run에 요구하지 않는다.

## 9. 새 Codex 세션에서 반드시 지킬 규칙
1. 먼저 이 문서를 읽고 시작할 것.
2. `FAST_LIO/`만 active 수정 대상으로 볼 것.
3. `prev/`는 절대 수정하지 말 것.
4. host-only ROS가 아니라 Docker Compose 흐름으로 build/run 할 것.
5. 지금 단계는 연구 novelty 확정이 아니라 **engineering stabilization step** 으로 다룰 것.
6. FAST-LIO backbone, EKF, ikd-tree를 지금 단계에서 제거하지 말 것.
7. state-vector 확장도 지금 단계에서 하지 말 것.

## 10. 다음 권장 작업
다음 작업은 **추가 real bag들에 대해 hybrid mode를 검증**하는 것이다.

권장 비교 대상:
1. baseline
2. 이전 failed proposed
3. 현재 hybrid

bag마다 최소 확인 항목:
- runtime rows
- acceptance ratio
- raw signed distance 통계
- final pose
- `No Effective Points!` 발생 횟수
- support filtering이 너무 보수적인지 여부

## 11. 새 Codex 세션 시작용 짧은 프롬프트
아래 문장을 새 세션 첫 프롬프트로 사용하면 된다.

`docs/codex_session_handoff_ko.md`를 먼저 읽고 그 상태에서 이어서 작업해라.
현재 active code는 `FAST_LIO/`이고, `prev/`는 frozen baseline이므로 수정하지 마라.
기존 Docker Compose 환경(`docker-compose/ges-voxel_compose.yml`, service `ges-voxel`)을 사용하라.
현재 hybrid mode는 연구 최종 결론이 아니라 engineering stabilization step이다.
다음 작업은 추가 real bag들에서 hybrid integration을 baseline 및 이전 failed proposed와 비교 검증하는 것이다.
EKF, ikd-tree, backbone 전체 구조, state-vector를 이번 단계에서 바꾸지 마라.
