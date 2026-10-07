# Results

Each folder is one pipeline run (`scripts/run_pipeline.sh`). `REPORT.md` has the verdict and every check; `report.json`
has the full evidence (actions with parameters, mass/inertia with their basis, sweep rows); `sweep.png` plots max
penetration against joint angle; `sweep.mp4` is the MuJoCo render of the sweep with contact points; `preview.png` is a
Cycles render of the fixed-up mesh.

| run | input | verdict |
|---|---|---|
| `clean` | synthetic box, two parts, shipped at 100x scale | PASS |
| `fused` | same box exported as ONE mesh (split at the seam by the pipeline) | PASS |
| `wrong-scale` | rescale deliberately skipped | FAIL |
| `lid-through-box` | hinge pivot moved 5 cm forward | FAIL |
| `wrong-axis` | hinge axis Y instead of X | FAIL |
| `hy3d` | real Hunyuan3D-2 mesh, lid modelled open (`samples/hy3d/white_mesh.glb`) | FAIL: lid skirt hits the side walls, 25 mm; `pivot_calibration.json` shows no pivot within +-3 cm of the knuckles fixes it |
| `hy3d_fixed` | same mesh, joint moved to the best-fit **virtual** pivot (3 cm inside, 6 cm above the knuckles) and travel cut to -20..68 deg | PASS (2.5 mm): the generated lid only articulates about a hinge that is not where it drew the hinge |
| `hy3d_closed` | second generation from a closed-lid reference photo (`samples/hy3d/closed_mesh.glb`), hinge on the lid's back edge, sweep 0..-110 deg | see `hy3d_closed/REPORT.md` |

`hy3d/asset_physics.usdc` is the UsdPhysics-authored stage (Z-up, metres), `hy3d/asset.xml` the MJCF, and
`hy3d/mjcf_meshes/` the CoACD hulls it references.
