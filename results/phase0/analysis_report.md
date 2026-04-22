# Phase 0 코드 구조 분석 및 수정 계획

## 1. 이번 단계 목표
- `FAST_LIO/` 내부에서 primitive / residual / map update 핵심 경로를 찾는다.
- `prev/FAST_LIO`와 `prev/BALM`을 비교 참조하여 최소 침습 수정 포인트를 정한다.

## 2. 확인한 핵심 파일/함수
- `FAST_LIO/src/laserMapping.cpp`
  - `lasermap_fov_segment()`: local map support 유지
  - `h_share_model()`: point-to-map residual / correspondence / Jacobian 계산 핵심
  - `map_incremental()`: map 삽입 경로
- `FAST_LIO/include/common_lib.h`
  - `esti_plane()`: baseline plane fitting helper
- `FAST_LIO/src/preprocess.cpp`
  - front-end feature 처리 경로
- `FAST_LIO/config/*.yaml`, `FAST_LIO/launch/*.launch`
  - 런타임 파라미터와 launch 진입점

## 3. `prev/FAST_LIO` 비교 요약
- residual / correspondence / map update의 핵심 경로는 현재 `FAST_LIO`와 실질적으로 동일했다.
- 따라서 baseline은 현재 `FAST_LIO`를 직접 기준선으로 사용해도 무방하다고 판단했다.

## 4. `prev/BALM` 참고 포인트
- local BA는 recent pose window를 유지하며 pose 갱신 후 residual을 재평가하는 구조를 가진다.
- FAST_LIO에는 처음부터 BA 전체를 넣지 않고,
  - recent keyframe buffer
  - local refinement entry point
  - pose update 후 residual 재평가 hook
  수준만 남기는 것이 최소 침습에 맞다.

## 5. 수정 포인트 파일 목록
- `FAST_LIO/src/laserMapping.cpp`
- `FAST_LIO/include/primitive_residual_models.hpp` (신규)
- `FAST_LIO/include/local_refinement_hook.hpp` (신규)
- `FAST_LIO/src/primitiveBenchmark.cpp` (신규)
- `FAST_LIO/CMakeLists.txt`
- `FAST_LIO/config/*.yaml`

## 6. 함수 수준 수정 계획
- `laserMapping.cpp::h_share_model()`
  - baseline / proposed primitive-residual 분기 추가
  - residual 통계 수집 추가
- `laserMapping.cpp::map_incremental()`
  - baseline map insertion semantics 유지
- `main()`
  - primitive 관련 YAML/ROS param 로딩
- 별도 helper
  - baseline plane residual
  - proposed surface-aware Lp proxy residual

## 7. 위험 요소 목록
- FAST-LIO 실제 baseline은 point-to-plane 계열이라, 원문의 Gaussian/center-based 표현과 직접 일치하지 않을 수 있다.
- 실데이터 bag 없이는 trajectory / map fidelity / runtime 검증이 incomplete 상태로 남는다.
- proposed residual은 정규화가 들어가므로 raw residual 크기 비교를 그대로 해석하면 왜곡될 수 있다.
- neighbor 설정이 map insertion까지 건드리면 residual 효과와 map density 효과가 섞일 위험이 있다.

## 8. 단계별 구현 계획
1. baseline build/run 확보
2. baseline/proposed 전환 스위치 추가
3. proposed primitive 최소 proxy 구현
4. synthetic local benchmark로 로컬 비교
5. 실데이터 bag 확보 후 full pipeline 비교
6. 그 다음에만 local BA hook를 실제 활성화
