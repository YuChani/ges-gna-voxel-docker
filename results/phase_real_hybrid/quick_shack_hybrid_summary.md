# Real Bag Hybrid Stabilization Summary

## Run Setup
- Container workflow: `docker-compose/ges-voxel_compose.yml`, service/container `ges-voxel`
- Workspace: `/root/catkin_ws`
- Bag: `/root/dataset/sensor_data/livox_avia/2020-09-16-quick-shack.bag`
- Output: `results/phase_real_hybrid/quick_shack_hybrid/`
- Active mode: `hybrid_plane_surface`

## Hybrid Result
- Runtime rows: `487`
- Odometry rows: `487`
- Mean acceptance ratio: `0.485036`
- Raw signed distance mean/std/max: `0.016211 / 0.017979 / 0.380591`
- Surface score mean/std: `0.925325 / 0.070182`
- Tangent LP mean/std: `0.379706 / 0.322374`
- Planarity mean/std: `0.997621 / 0.003748`
- Final pose: `(-0.039334, -0.026460, 0.055136)`
- `No Effective Points!` count: `0`

## Previous Proposed Failure Reference
- Reference path: `results/phase_real_compose/quick_shack_proposed/`
- Runtime rows: `487`
- Mean acceptance ratio: `0.254161`
- Raw signed distance mean/std/max: `0.013936 / 0.011386 / 0.122779`
- Tangent LP mean: `0.904170`
- Planarity mean: `0.947268`
- Final pose: `(-135.247693, -135.089559, 13.879704)`
- `No Effective Points!` count: `4`

## Baseline Context
- Reference path: `results/phase_real_compose/quick_shack_baseline/`
- Runtime rows: `487`
- Mean acceptance ratio: `0.689416`
- Raw signed distance mean/std/max: `0.023254 / 0.027287 / 0.386514`
- Final pose: `(-0.067851, -0.050983, 0.070171)`

## Notes
- Hybrid no longer uses the normalized surface residual as the EKF residual or Jacobian direction.
- The remaining acceptance gap versus baseline is expected because hybrid also applies support-quality filtering.
- This is an integration stabilization result, not a final research-positioning claim.
