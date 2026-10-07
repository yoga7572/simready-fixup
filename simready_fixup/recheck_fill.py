"""Recompute the collision_proxy_fill check for an existing run from its hull OBJs (no CoACD rerun)."""
import glob, json, os, sys
import numpy as np, trimesh
from simready_fixup.stage_mjcf import material_volume
outdir, spec_path = sys.argv[1], sys.argv[2]
spec = json.load(open(spec_path)); wt = float(spec.get("wall_thickness_m", 0.003)); limit = float(spec.get("collision_fill_limit", 4.0))
rep = json.load(open(os.path.join(outdir, "report_mjcf.json"))); bad = []
for act in rep["actions"]:
    if act["id"] != "coacd": continue
    name = act["part"]; tm = trimesh.load(os.path.join(outdir, f"{name}.obj"), force="mesh")
    hv = sum(abs(float(trimesh.load(p, force="mesh").volume)) for p in glob.glob(os.path.join(outdir, "mjcf_meshes", f"{name}_hull*.obj")))
    mv, basis = material_volume(tm, wt)
    act.update({"hull_volume_m3": round(hv, 5), "material_volume_m3": round(mv, 5), "material_basis": basis,
                "hull_over_material": round(hv / max(mv, 1e-12), 5), "hull_over_bbox": round(hv / max(float(np.prod(tm.extents)), 1e-12), 5)})
    act.pop("hull_volume_over_bbox", None)
    if act["hull_over_material"] > limit: bad.append((name, round(act["hull_over_material"], 1)))
rep["checks"] = [c for c in rep["checks"] if c["id"] != "collision_proxy_fill"]
rep["checks"].insert(0, {"id": "collision_proxy_fill", "ok": not bad, "parts_over_limit": bad, "limit": limit,
    "note": "sum of hull volumes / material volume (closed mesh volume, or area x wall thickness / 2 for open shells); far above 1 means the decomposition sealed a cavity (non-manifold input or too coarse preprocess_resolution) and the sweep tested a solid block"})
json.dump(rep, open(os.path.join(outdir, "report_mjcf.json"), "w"), indent=2)
print(os.path.basename(outdir), "fill:", [(a["part"], a["hulls"], a["hull_over_material"]) for a in rep["actions"] if a["id"] == "coacd"], "bad:", bad)
