# 제안 아이디어 vs FAST-LIO2 baseline 상세 분석

## 1. 문서 목적

이 문서는 현재 구현된 제안 방법이 왜 FAST-LIO2 baseline보다 불안정하게 동작했는지, 그리고 그 현상이 **아이디어 자체의 실패인지**, 아니면 **현재 FAST-LIO2 backbone에 통합한 방식의 문제인지**를 분리해서 설명하기 위한 상세 분석 문서다.

핵심 질문은 다음과 같다.

1. 내가 제안한 surface-aware primitive / point-to-surface residual 아이디어는 현재 어떤 형태로 구현되었는가?
2. FAST-LIO2 baseline은 실제로 어떤 측정 모델과 local map 가정을 쓰는가?
3. 현재 실험에서 baseline보다 안 좋게 나온 직접적인 증상은 무엇인가?
4. 그 원인은 residual 설계, EKF integration, ikd-tree local support 사용 방식 중 어디에 있는가?
5. 따라서 무엇을 “아이디어의 문제”로 보고, 무엇을 “현재 integration 방식의 문제”로 봐야 하는가?

---

## 2. 현재 비교 대상 정리

### 2.1 FAST-LIO2 baseline (`primitive_mode=0`)

현재 코드베이스에서 baseline은 논문 설명상의 “Gaussian/NDT center-based”가 아니라, **실제 FAST-LIO2가 사용하는 point-to-plane residual 기반 local scan-to-map matching**이다.

- local support: `ikdtree.Nearest_Search(...)`로 얻은 최근접 이웃
- primitive: 최근접 이웃으로부터 추정한 local plane
- residual: point-to-plane signed distance
- Jacobian direction: plane normal

관련 코드:
- `FAST_LIO/src/laserMapping.cpp`
- `FAST_LIO/include/primitive_residual_models.hpp::ComputePlaneResidual`

### 2.2 제안 방법 (`primitive_mode=1`)

현재 구현된 proposed는 “surface-aware primitive + point-to-surface consistency residual”의 **최소 proxy 구현**이다.

- local support: baseline과 동일하게 `ikdtree`의 최근접 이웃 사용
- primitive: local covariance 기반 tangent-normal frame + axis scale
- shape parameter: `p`를 쓰는 Lp-type tangent gate
- residual:
  - local normal direction signed distance를 계산하고
  - tangent 위치에 따라 gate를 걸어 정규화된 residual을 만듦
- Jacobian direction:
  - normal gradient
  - tangent gate 변화량
  을 합친 근사 gradient

관련 코드:
- `FAST_LIO/include/primitive_residual_models.hpp::ComputeSurfaceLpResidual`

### 2.3 center-based Gaussian proxy baseline (`primitive_mode=2`)

추가로 연구 설명의 center-based baseline 축을 보존하기 위해 center Gaussian proxy baseline도 넣어 두었다.

- primitive: centroid + covariance scale 기반 ellipsoidal model
- residual: 중심 기반 ellipsoid level-set residual

이 모드는 연구적 비교축을 맞추기 위한 proxy이고, FAST-LIO2 실제 baseline은 아니다.

---

## 3. 제안 아이디어의 원래 의도

제안 아이디어의 핵심은 단순히 norm을 바꾸는 것이 아니다.

의도는 다음과 같았다.

1. LiDAR point는 volumetric sample이 아니라 대체로 surface sample이다.
2. 따라서 voxel / local support 내부를 단순한 center-based blob으로 보는 것은 edge, boundary, corner, mixed structure에서 비효율적일 수 있다.
3. local primitive를 shape-adaptive하게 만들면 local geometry를 더 잘 표현할 수 있다.
4. 그 primitive에 대해 point-to-surface consistency residual을 쓰면 일부 구조에서 더 informative한 alignment signal을 줄 수 있다.
5. 결과적으로 local map fidelity와 alignment robustness를 높일 수 있다는 것이 가설이었다.

이 아이디어 자체는 geometry 관점에서 충분히 타당한 문제 제기다. 다만 현재 결과는 **그 아이디어를 FAST-LIO2의 online measurement path에 직접 넣었을 때 오히려 더 불안정해졌다**는 것을 보여준다.

---

## 4. 현재 구현이 실제로 들어간 위치

현재 proposed는 FAST-LIO2 전체 pipeline을 뒤엎지 않고, 핵심 measurement path에 최소 침습으로 삽입되었다.

### 4.1 삽입 위치

- 핵심 함수: `FAST_LIO/src/laserMapping.cpp::h_share_model()`

여기서 현재 수행되는 흐름은 다음과 같다.

1. 현재 scan point를 world frame으로 변환
2. `ikdtree.Nearest_Search(...)`로 local support 획득
3. `ComputePrimitiveResidual(...)` 호출
4. acceptance score threshold를 넘는 경우만 correspondence 채택
5. 채택된 residual / gradient를 EKF measurement에 직접 주입

즉, proposed는 “후처리”가 아니라 **FAST-LIO2 update의 중심 measurement path** 안으로 들어가 있다.

---

## 5. 현재 저장된 실험 결과

### 5.1 로컬 synthetic benchmark

관련 파일:
- `results/phase3/comparison_summary.md`
- `results/phase3/primitive_benchmark.csv`
- `results/phase3/primitive_benchmark.log`

핵심 관찰:

- held-out fitting error는 baseline plane residual이 더 작음
  - plane patch: baseline `0.021264`, proposed `0.193153`
  - edge patch: baseline `0.025629`, proposed `0.214244`
  - corner patch: baseline `0.034207`, proposed `0.392915`

- 하지만 tangent perturbation 일부에서는 proposed가 더 작은 residual을 보이는 경우가 있음
  - 예: plane patch tangent translation `-0.30`
    - baseline `0.206348`
    - proposed `0.022100`

해석:

- proposed는 일부 tangent-sensitive 상황에서는 더 예민하게 반응할 수 있다.
- 하지만 online odometry에서 중요한 건 “특정 perturbation 민감도”보다 “전체 측정 안정성”인데, 그 점에서는 baseline이 우세했다.

### 5.2 synthetic full-pipeline

관련 파일:
- `results/phase3/synthetic_full_pipeline_summary.md`

핵심 수치:

| 항목 | baseline | proposed |
|---|---:|---:|
| mean acceptance ratio | 0.9945 | 0.5488 |
| trajectory XY error mean | 0.3883 | 10.6497 |
| front wall thickness mean abs | 0.0267 | 3.8193 |

해석:

- synthetic에서도 proposed는 baseline보다 correspondence를 적게 채택했다.
- pose drift가 커졌고 map wall thickness도 크게 증가했다.
- 즉 synthetic full-pipeline 단계에서 이미 “proposal이 online odometry를 불안정하게 만들 가능성”이 드러났다.

### 5.3 real bag replay (compose 기준)

관련 파일:
- `results/phase_real_compose/quick_shack_report.md`
- `results/phase_real_compose/quick_shack_baseline/`
- `results/phase_real_compose/quick_shack_proposed/`

사용 bag:
- `/root/dataset/sensor_data/livox_avia/2020-09-16-quick-shack.bag`

핵심 수치:

| 항목 | baseline | proposed |
|---|---:|---:|
| runtime rows | 487 | 487 |
| mean acceptance ratio | 0.6894 | 0.2542 |
| mean signed_dist_abs_mean | 0.02325 | 0.01394 |
| mean search time (s) | 0.001420 | 0.001645 |
| mean incremental time (s) | 0.000066 | 0.000379 |
| final pose x | -0.0679 | -135.2477 |
| final pose y | -0.0510 | -135.0896 |
| final pose z | 0.0702 | 13.8797 |

launch log 증상:
- `results/phase_real_compose/quick_shack_proposed/launch.log`
- 반복된 `No Effective Points!`

해석:

- real bag에서도 proposed는 baseline보다 correspondence acceptance가 크게 낮았다.
- 최종 pose가 크게 발산했고, 이는 단순히 “약간 안 좋다”가 아니라 **measurement path가 무너진 상태**에 가깝다.

---

## 6. 관찰된 주요 증상 정리

현재 구현에서 드러난 핵심 증상은 다음 네 가지다.

### 6.1 증상 A: correspondence acceptance 붕괴

baseline 대비 proposed에서 acceptance ratio가 크게 떨어졌다.

- synthetic: `0.9945 -> 0.5488`
- real bag: `0.6894 -> 0.2542`

이는 곧 좋은 대응점이 baseline보다 훨씬 덜 채택되었다는 뜻이다.

### 6.2 증상 B: `No Effective Points!` 반복

`launch.log`에서 proposed run은 `No Effective Points!`가 반복된다.

즉, 측정 업데이트에 사용할 유효 포인트 수가 특정 시점에 거의 고갈되었다.

### 6.3 증상 C: pose divergence

real bag final pose는 baseline이 거의 원점 근처인데 proposed는 크게 발산했다.

이는 residual이 조금 noisier한 정도가 아니라 **state update 자체가 잘못된 방향으로 누적되었다**는 뜻이다.

### 6.4 증상 D: map quality degradation

synthetic map-quality proxy에서 proposed는 wall thickness가 크게 증가했다.

즉, 제안 residual이 local map fidelity를 개선하기보다 오히려 악화시킨 흔적이 있다.

---

## 7. 원인 분석 표

| 원인 | 증거 | 해석 | 수정 방향 |
|---|---|---|---|
| **1. residual 스케일 변경과 gating 기준의 불일치** | `ComputeSurfaceLpResidual()`은 `signed_distance / (normal_scale * tangent_gate)`를 residual로 정의한다. 그런데 `laserMapping.cpp`에서는 이 residual로 그대로 acceptance를 결정한다. real bag에서 proposed는 baseline보다 `signed_dist_abs_mean`이 더 작지만 acceptance는 더 낮다. | geometric distance는 더 작아도, EKF가 보는 residual은 normalized 값이라 더 harsh해질 수 있다. 즉 point rejection이 “geometry 때문”이 아니라 “스케일 때문에” 일어난다. | accept/reject는 raw signed distance 기준으로 분리하고, surface-aware residual은 optimization score로만 쓰는 hybrid 구조 검토 |
| **2. Jacobian이 baseline point-to-plane보다 복잡하고 불안정함** | baseline은 plane normal 기반 direction을 쓰지만, proposed는 `world_gradient = normal_grad - tangent_grad`를 사용하고 이를 EKF Jacobian에 직접 넣는다. | FAST-LIO2 EKF는 point-to-plane 구조에 맞춰져 있는데, proposed는 tangent effect까지 measurement 방향에 포함시켜 conditioning을 흔든다. | update direction은 더 보수적으로 두고, tangent term은 residual modulation이나 score 단계로 제한 |
| **3. ikd-tree의 5-NN support를 proposed가 더 민감하게 소비함** | baseline은 최근접 이웃으로 local plane을 만들고 support threshold를 검사한다. proposed는 같은 5-NN에서 PCA frame/scale/tangent gate를 모두 추정한다. | ikd-tree가 문제라기보다, same local support를 proposal이 훨씬 더 고차원적으로 해석하므로 mixed structure / boundary / noise에 더 민감해진다. | neighbor 수 증가, planarity gating 추가, support consistency 체크 강화 |
| **4. correspondence starvation 이후 pose/map 붕괴의 악순환** | acceptance 붕괴 → `No Effective Points!` → pose drift → 다음 frame의 neighbor quality 저하 → 다시 acceptance 하락. synthetic와 real bag 모두 이 패턴이 보인다. | 제안 residual이 좋은 대응점을 너무 많이 잃으면서 update가 약해지고, 그 결과 next scan matching도 더 나빠지는 self-reinforcing failure가 생긴다. | 우선 acceptance ratio를 회복시키는 방향으로 residual/gating부터 수정 |
| **5. local shape sensitivity는 생겼지만 online odometry 목적과 어긋남** | tangent perturbation 일부에서는 proposed가 좋아 보이지만, held-out fitting / full-pipeline trajectory / map quality는 전반적으로 악화된다. | proposal은 “특정 구조에서 더 예민한 signal”은 만들 수 있어도, 현재 구현은 odometry 전체 안정성보다 그 민감도를 더 키우는 방향으로 작동한다. | surface-aware 정보는 먼저 reweighting / refinement 단계에 쓰고, main EKF update는 baseline point-to-plane을 유지하는 hybrid 구조 고려 |

---

## 8. “EKF와 충돌했다”는 표현의 정확한 의미

이걸 단순히 “EKF가 제안 방법을 못 받아들였다”라고 말하면 조금 부정확하다.

더 정확한 표현은 다음과 같다.

> 제안 residual/Jacobian이 FAST-LIO2 EKF가 원래 기대하는 **point-to-plane residual의 스케일, 방향, 안정성 가정**과 맞지 않게 들어갔다.

즉,

- baseline FAST-LIO2는
  - local plane
  - signed point-to-plane distance
  - simple plane normal Jacobian
  - robust gating
  구조다.

- proposed는
  - tangent gate가 들어간 normalized residual
  - tangent term이 포함된 gradient
  - same threshold
  를 쓴다.

결국 EKF가 받는 measurement의 성격이 달라졌는데, noise/gating/update 설계는 여전히 baseline 가정에 가까웠다.

따라서 “EKF와 충돌”은 맞지만, 그것은 알고리즘적 integration mismatch이지 소프트웨어 오류는 아니다.

---

## 9. “ikd-tree와 충돌했다”는 표현의 정확한 의미

ikd-tree 자체가 제안 방법과 충돌했다고 보기보다는,

> baseline이 robust하게 쓰던 5-NN local support를 proposed가 더 민감하고 복잡한 primitive fitting에 사용하면서 support quality 문제가 더 크게 드러났다.

라고 보는 것이 맞다.

즉 ikd-tree는 단지 local support를 빠르게 제공하는 자료구조다. 문제는 그 support를 어떻게 해석하느냐인데,

- baseline은 그 support에서 plane을 만들고 잘 안 맞으면 버림
- proposed는 그 support에서 centroid, covariance frame, scales, tangent gate를 모두 추정함

따라서 same neighbor set이라도 proposal이 더 쉽게 흔들린다.

---

## 10. 현재 결론

현재 증거만 기준으로 하면, 다음 해석이 가장 타당하다.

1. **제안 아이디어 자체를 완전히 폐기할 정도의 증거는 아직 아니다.**
   - tangent-sensitive 상황에서 proposal이 더 informative한 응답을 보인 경우는 있다.

2. 하지만 **현재 FAST-LIO2 backbone에 직접 넣은 integration 방식은 baseline보다 분명히 안 좋다.**
   - synthetic, real bag 둘 다에서 correspondence acceptance가 크게 떨어지고
   - trajectory와 map fidelity가 악화되었다.

3. 따라서 지금의 결론은

> “아이디어가 틀렸다” 보다는,
> “현재 residual/Jacobian/gating integration 방식이 FAST-LIO2 measurement 구조와 잘 맞지 않아 오히려 불안정해졌다”

에 가깝다.

---

## 11. 다음 수정 우선순위 제안

### 1순위
acceptance gate를 normalized residual이 아니라 raw signed distance 기준으로 분리할 것.

### 2순위
surface-aware residual을 쓰더라도 EKF update direction은 baseline normal-like 구조에 더 가깝게 제한할 것.

### 3순위
proposal 모드에서는 support quality를 더 엄격하게 필터링할 것.
- neighbor 수 증가
- planarity threshold 추가
- mixed structure reject 조건 추가

### 4순위
accepted points의 `normal_scale`, `tangent_lp`, `planarity` 분포를 frame-by-frame으로 분석해 collapse 시점을 추적할 것.

### 5순위
surface-aware 정보는 먼저 main EKF path가 아니라 reweighting / local refinement / post-update refinement 단계에서 사용하는 hybrid 구조를 검토할 것.

---

## 12. 참고 파일

- `results/final_report.md`
- `results/phase_real_compose/quick_shack_report.md`
- `results/phase_real_compose/quick_shack_proposed/launch.log`
- `results/phase3/comparison_summary.md`
- `results/phase3/synthetic_full_pipeline_summary.md`
- `FAST_LIO/include/primitive_residual_models.hpp`
- `FAST_LIO/src/laserMapping.cpp`
