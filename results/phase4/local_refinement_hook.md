# Phase 4 local refinement hook 설계 메모

## 1. 이번 단계 목표
- BALM 전체를 가져오지 않고, FAST_LIO 안에 나중에 local refinement를 꽂을 수 있는 최소 hook만 준비한다.

## 2. 확인한 핵심 파일/함수
- `FAST_LIO/include/local_refinement_hook.hpp`
- `FAST_LIO/src/laserMapping.cpp`
- 참고 레퍼런스: `prev/BALM/BALM-old/src/balmclass.hpp`, `balm_only_back.cpp`, `balm_front_back.cpp`

## 3. 수정한 파일 목록
- `FAST_LIO/include/local_refinement_hook.hpp`
- `FAST_LIO/src/laserMapping.cpp`
- `FAST_LIO/config/*.yaml`

## 4. 구현한 내용
- `LocalRefinementConfig`
  - `enable`
  - `recent_keyframe_limit`
  - `local_window_size`
- `RecentKeyframeState`
  - timestamp
  - pose
  - downsampled point count
  - effective correspondence count
- `LocalRefinementHook`
  - 최근 keyframe ring-buffer 보관
  - 최소 window 크기 체크
  - 최근 window 반환
- `ReevaluateRecentWindow()`
  - 최근 window의 residual mean / correspondence mean을 재평가하는 최소 entry point
- `RefineLatestKeyframePose()`
  - recent window anchor를 기준으로 latest keyframe translation을 소폭 보정하는 pose-only refinement entry
- `ReevaluateLatestProxyError()`
  - pose update 직후 latest keyframe proxy residual을 다시 평가하는 후속 check
- 현재는 기본적으로 disabled이며 baseline 동작을 바꾸지 않는다.
- `laserMapping.cpp`에서 keyframe push 직후 이 entry point를 실제로 호출한다.

## 5. 빌드/실행 결과
- baseline/proposed build 모두 성공
- smoke test에서 hook 추가로 인한 startup regression 없음

## 6. 실험 또는 로그 결과
- 현재 hook는 여전히 BA optimizer는 아니지만, recent window 재평가, pose-only translation refinement, pose update 직후 residual recheck가 실제 online 경로에 연결되었다.

## 7. 남은 문제
- full BA는 아직 없음
- voxel dirty set 기반 재구성이나 더 강한 local pose refinement는 차후 구현 필요

## 8. 다음 단계 제안
- `h_share_model()` 이후, `map_incremental()` 이전에 optional local refine entry 삽입
- recent keyframe window에 대해 proposed residual만 재평가하는 작은 optimizer 추가
- old frame는 BALM처럼 marginal summary만 유지하는 구조 검토
