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
| `hy3d` | real Hunyuan3D-2 mesh (`samples/hy3d`) | FAIL: lid skirt hits the side walls, 25 mm; `pivot_calibration.json` shows no nearby pivot fixes it |

`hy3d/asset_physics.usdc` is the UsdPhysics-authored stage (Z-up, metres), `hy3d/asset.xml` the MJCF, and
`hy3d/mjcf_meshes/` the CoACD hulls it references.
