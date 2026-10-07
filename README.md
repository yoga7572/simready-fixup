# simready-fixup

Turn an AI-generated GLB into a simulation-ready asset — and prove it with a report that can say **FAIL**.

**Status (2026-10-07):** runs end to end on two assets.

| asset | source | verdict | why |
|---|---|---|---|
| `samples/box_lid_*.glb` | synthetic lidded box, shipped at 100x scale on purpose | **PASS** | scale fixed from the spec, hinge sweep clean (2.2 mm, CoACD hull overshoot) |
| same, `--inject wrong-scale / lid-through-box / wrong-axis` | injected defects | **FAIL** x3 | the validator catches each one |
| `samples/hy3d/white_mesh.glb` | **real AI mesh**: Hunyuan3D-2 (HF Space) from a FLUX.1-schnell reference photo, 724k faces, 15k loose shells, lid modelled open | **FAIL** | after fragment removal, decimation to 60k faces, width-based rescale and a seam cut at the rim, the lid cannot close: 25 mm interference between the lid skirt and the side walls. `calibrate_pivot` shows no hinge position within +-3 cm of the drawn knuckles fixes it, so it is the generator's geometry, not the annotation |
| same mesh, `samples/hy3d_box_fixed_spec.json` | fix-up: joint moved to the best-fit **virtual** pivot (3 cm inside, 6 cm above the knuckles), travel -20..68 deg | **PASS** (2.5 mm) | the only rigid fix that works; documented in the spec as a simulation-only hinge, because a real hinge cannot sit there |
| `samples/hy3d/closed_mesh.glb` | second generation from a **closed-lid** reference photo (1.09M faces, 524k loose shells) | see `results/hy3d_closed` | the workflow Palatial-style pipelines prefer: model closed, cut at the seam, articulate, validate opening |

Nothing here has been run inside Isaac Sim yet — see *Limitations*.

## What it does

```
GLB  ──▶  stage 1  bpy (Blender 5.2, headless)  ──▶  asset_visual.usda + per-part OBJ + report_bpy.json
            inspect: parts, bounding box, scale vs. spec, watertightness
            fix-up:  apply transforms, rescale to the spec dimensions, Z-up, origin at base centre, part naming
      ──▶  stage 2  usd-core 26.8                ──▶  asset_physics.usda + report_usd.json
            UsdPhysics: Scene, ArticulationRoot, RigidBodyAPI, CollisionAPI (convexDecomposition),
            MassAPI (mass / COM / inertia from watertight components), RevoluteJoint with limits
            validation: UsdValidation (28 registered validators) + built-in physics sanity checks
      ──▶  stage 3  CoACD + MuJoCo 3.15           ──▶  asset.xml (MJCF) + mjcf_meshes/ + report_mjcf.json
            convex decomposition per part, MJCF with parent-child collision filtering DISABLED,
            joint sweep through the full range, max penetration per angle
      ──▶  report.json / REPORT.md with a single PASS / FAIL verdict
```

Stage 1 runs in its own process because the `bpy` wheel ships its own USD build; stage 2 uses pip `usd-core`.

## Run

```bash
uv sync                                                   # Python 3.13: bpy, usd-core, coacd, mujoco, trimesh, scipy
uv run python -m simready_fixup.synth samples/box_lid_raw.glb --spec samples/box_lid_spec.json
scripts/run_pipeline.sh samples/box_lid_raw.glb samples/box_lid_spec.json out/clean
scripts/run_pipeline.sh samples/box_lid_raw.glb samples/box_lid_spec.json out/bad --inject lid-through-box
```

`--inject` accepts `wrong-scale`, `lid-through-box`, `wrong-axis`. Each must produce a `FAIL` — that is the test of the
validator, not of the asset.

Each run leaves `REPORT.md`, `sweep.png` (angle vs. max penetration) and `sweep.mp4` (MuJoCo off-screen render of the
sweep with the live penetration value burned in; needs `MUJOCO_GL=glfw` on macOS, which `run_pipeline.sh` sets).

The real-asset run: `scripts/run_pipeline.sh samples/hy3d/white_mesh.glb samples/hy3d_box_spec.json out/hy3d`, then
`uv run python -m simready_fixup.calibrate_pivot out/hy3d samples/hy3d_box_spec.json` to check whether a FAIL is an
annotation error (a nearby pivot clears it) or a genuine interference (nothing clears it).

Generating your own input: `gen_tripo.py` (needs `TRIPO_API_KEY` in `.env`), or any image-to-3D service; the
`samples/hy3d` asset came from the public Hugging Face Spaces `black-forest-labs/FLUX.1-schnell` (reference image) and
`tencent/Hunyuan3D-2` (shape generation), no account needed. Then write the matching `samples/<name>_spec.json` by hand
(dimensions and their basis, density, seam height for `split`, joints).

## What the fix-up stage actually does to an AI mesh

`report_bpy.json` lists every action with its parameters: `remove_fragments` (drop shells under `min_component_faces`),
`decimate` (collapse to `face_budget`), `rescale` (one spec dimension drives the factor, `scale_axis`), `recenter_origin`
(base centre at the origin, Z-up), `split_plane` (bisect a fused mesh at the hand-annotated seam and cap the cut with a
ring so hollow parts stay hollow), `rename_parts`. `preview.png` is a Cycles render of the result.

## The spec is the source of truth

AI generators do not know real-world size, mass or how an object moves. `samples/*_spec.json` carries what the mesh
cannot: target dimensions and where they come from (prompt / datasheet), a density assumption, and **hand-annotated
joints** (parent, child, axis, origin in the asset frame, limits, and the reasoning). Every number the pipeline derives
from these is written back into the report together with its basis.

## Checks and what a FAIL means

| check | stage | fails when |
|---|---|---|
| `scale_after_fix` | bpy | bounding-box height differs from the spec by more than 5 % after fix-up |
| `stage_metadata` | usd | `upAxis` is not `Z` or `metersPerUnit` is not 1 |
| `usd_validation` | usd | any UsdValidation error, or a part without RigidBodyAPI / CollisionAPI / positive mass, or a joint with invalid bodies or limits |
| `joint_sweep` | mjcf | max penetration between parts over the joint range exceeds the tolerance (default 3 mm) |

Known tolerance source: CoACD hulls overshoot the surface by roughly 2 mm on this asset (`hull_bbox_overshoot_mm` in
the report), so the tolerance cannot be tighter than that without better decomposition settings.

## Tooling note

Built in a few days by one person using an AI coding agent (Claude Code) to drive headless Blender, usd-core, CoACD and
MuJoCo. Every number in the reports comes from those tools, never from the model; the agent writes the scripts, the
simulator decides. The `samples/hy3d` mesh is a Hunyuan3D-2 output (Tencent Hunyuan 3D 2.0 Community License); the
reference photo is a FLUX.1-schnell output.

## Limitations (read before trusting a PASS)

- **MuJoCo pass is not an Isaac Sim pass.** Collision filtering, solver and joint conventions differ; the USD carries the
  UsdPhysics schemas Isaac Sim expects, but it has not been loaded there yet.
- Joints are hand-annotated, not inferred. The validator catches a wrong annotation; it does not propose the right one.
- Materials and textures are passed through untouched; UVs are not checked.
- Convex decomposition over-approximates thin parts; a tight lid will register a few millimetres of contact at rest.
- Only revolute joints; one asset in the sample set.
