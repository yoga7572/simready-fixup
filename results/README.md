# Results

Each folder is one pipeline run (`scripts/run_pipeline.sh`). `REPORT.md` has the verdict and every check; `report.json`
has the full evidence (actions with parameters, mass/inertia with their basis, sweep rows); `sweep.png` plots max
penetration against joint angle; `sweep.mp4` is the MuJoCo render of the sweep with contact points; `preview.png` is a
Cycles render of the fixed-up mesh.

| run | input | verdict | failing check |
|---|---|---|---|
| `clean` | synthetic box, two parts, shipped at 100x scale | PASS | — |
| `fused` | same box exported as ONE mesh, cut at the seam by the pipeline | PASS | — |
| `wrong-scale` | rescale deliberately skipped | FAIL | `scale_after_fix`, `joint_sweep` |
| `lid-through-box` | hinge pivot moved 5 cm forward | FAIL | `joint_sweep` (18 mm) |
| `wrong-axis` | hinge axis Y instead of X | FAIL | `joint_sweep` (19 mm) |
| `hy3d` | real Hunyuan3D-2 mesh, lid modelled open (`samples/hy3d/white_mesh.glb`) | FAIL | `joint_sweep`: lid skirt hits the side walls, 25 mm; `pivot_calibration.json` shows no pivot within +-3 cm of the knuckles fixes it |
| `hy3d_fixed` | same mesh, joint moved to the best-fit **virtual** pivot (3 cm inside, 6 cm above the knuckles), travel -20..68 deg | PASS (2.5 mm) | — |
| `hy3d_closed` | second generation from a closed-lid photo (`samples/hy3d/closed_mesh.glb`), hinge on the lid's back edge, sweep 0..-110 deg | FAIL | `collision_proxy_fill`: the open shells cannot be closed, CoACD seals the cavity (hulls = 35x material volume), so the clean sweep is meaningless |
| `hy3d_closed_r160` | same with `coacd_preprocess_resolution` 160 | FAIL | same (35x) — resolution is not the fix, remeshing is |

`hy3d*/asset_physics.usdc` are the UsdPhysics-authored stages (Z-up, metres), `asset.xml` the MJCF and `mjcf_meshes/`
the CoACD hulls it references (kept for the two open-lid runs).
