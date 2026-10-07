"""Stage 3: CoACD convex decomposition -> MJCF -> MuJoCo joint sweep with parent-child collision filtering disabled.

Usage: python -m simready_fixup.stage_mjcf outdir spec.json [--inject lid-through-box|wrong-axis] [--tol-mm 1.0]
"""
from __future__ import annotations
import argparse, json, os
import numpy as np
import trimesh, coacd, mujoco

def decompose(obj_path, outdir, prefix, offset, threshold=0.05):
    tm = trimesh.load(obj_path, force="mesh")
    parts = coacd.run_coacd(coacd.Mesh(np.asarray(tm.vertices, dtype=np.float64), np.asarray(tm.faces, dtype=np.int64)), threshold=threshold)
    files = []
    for i, (v, f) in enumerate(parts):
        hull = trimesh.Trimesh(np.asarray(v) - offset, np.asarray(f), process=False)
        p = os.path.join(outdir, f"{prefix}_hull{i:02d}.obj"); hull.export(p); files.append(os.path.basename(p))
    return files, len(parts)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("outdir"); ap.add_argument("spec")
    ap.add_argument("--inject", choices=["lid-through-box", "wrong-axis"], default=None)
    ap.add_argument("--tol-mm", type=float, default=3.0, help="CoACD hulls overshoot the surface by ~2 mm; see hull_overshoot_mm")
    ap.add_argument("--steps", type=int, default=45)
    a = ap.parse_args()
    spec = json.load(open(a.spec)); j = spec["joints"][0]
    meshdir = os.path.join(a.outdir, "mjcf_meshes"); os.makedirs(meshdir, exist_ok=True)
    origin = np.array(j["origin_m"], dtype=float); axis = np.array(j["axis"], dtype=float)
    rep = {"stage": "mjcf", "checks": [], "actions": []}
    if a.inject == "lid-through-box": origin = origin + np.array([0, -0.05, 0]); rep["actions"].append({"id": "INJECT_lid_through_box"})
    if a.inject == "wrong-axis": axis = np.array([0, 1, 0.0]); rep["actions"].append({"id": "INJECT_wrong_axis"})
    parent, child = j["parent"], j["child"]
    hulls = {}
    for name, off in ((parent, np.zeros(3)), (child, origin)):
        files, n = decompose(os.path.join(a.outdir, f"{name}.obj"), meshdir, name, off)
        src = trimesh.load(os.path.join(a.outdir, f"{name}.obj"), force="mesh")
        hb = np.array([trimesh.load(os.path.join(meshdir, f), force="mesh").bounds for f in files]) + off
        overshoot = float(max((src.bounds[0] - hb[:, 0]).max(), (hb[:, 1] - src.bounds[1]).max(), 0.0)) * 1000.0
        hulls[name] = files; rep["actions"].append({"id": "coacd", "part": name, "hulls": n, "threshold": 0.05, "hull_bbox_overshoot_mm": overshoot})
        # visual mesh in the body frame too
        tm = trimesh.load(os.path.join(a.outdir, f"{name}.obj"), force="mesh"); tm.apply_translation(-off); tm.export(os.path.join(meshdir, f"{name}_visual.obj"))
    assets = "\n".join(f'    <mesh name="{f[:-4]}" file="{f}"/>' for n in (parent, child) for f in hulls[n] + [f"{n}_visual.obj"])
    def geoms(n):
        g = "\n".join(f'      <geom name="{f[:-4]}" type="mesh" mesh="{f[:-4]}" group="3" rgba="0.2 0.6 1 0.3"/>' for f in hulls[n])
        return g + f'\n      <geom name="{n}_visual" type="mesh" mesh="{n}_visual" contype="0" conaffinity="0" group="1" rgba="0.8 0.8 0.8 1"/>'
    lo, hi = j["limits_deg"]
    xml = f"""<mujoco model="{spec['name']}">
  <compiler angle="degree" meshdir="mjcf_meshes"/>
  <option gravity="0 0 -9.81"><flag filterparent="disable"/></option>
  <visual><global offwidth="1280" offheight="720"/></visual>
  <asset>
{assets}
  </asset>
  <worldbody>
    <body name="{parent}" pos="0 0 0">
{geoms(parent)}
      <body name="{child}" pos="{origin[0]} {origin[1]} {origin[2]}">
        <joint name="{j['name']}" type="hinge" axis="{axis[0]} {axis[1]} {axis[2]}" range="{lo} {hi}" limited="true"/>
{geoms(child)}
      </body>
    </body>
  </worldbody>
</mujoco>
"""
    mj_path = os.path.join(a.outdir, "asset.xml"); open(mj_path, "w").write(xml)
    model = mujoco.MjModel.from_xml_path(mj_path); data = mujoco.MjData(model)
    jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, j["name"]); qadr = model.jnt_qposadr[jid]
    rows = []; worst = 0.0
    for deg in np.linspace(lo, hi, a.steps):
        data.qpos[qadr] = np.deg2rad(deg); mujoco.mj_forward(model, data)
        pen = 0.0; pairs = []
        for i in range(data.ncon):
            c = data.contact[i]
            if c.dist < 0:
                pen = max(pen, -c.dist)
                pairs.append((mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, c.geom1), mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, c.geom2)))
        worst = max(worst, pen); rows.append({"angle_deg": float(deg), "ncon": int(data.ncon), "max_penetration_mm": pen * 1000.0, "pairs": pairs[:4]})
    tol = a.tol_mm / 1000.0
    ok = worst <= tol
    rep["checks"].append({"id": "joint_sweep", "joint": j["name"], "range_deg": [lo, hi], "steps": a.steps, "filterparent": "disabled",
                          "worst_penetration_mm": worst * 1000.0, "tolerance_mm": a.tol_mm, "ok": ok,
                          "note": "MuJoCo pass is not an Isaac Sim pass; convex hulls over-approximate thin parts"})
    rep["sweep"] = rows; rep["mjcf"] = mj_path
    json.dump(rep, open(os.path.join(a.outdir, "report_mjcf.json"), "w"), indent=2)
    print(json.dumps(rep["checks"], indent=1))
    bad = [r for r in rows if r["max_penetration_mm"] > a.tol_mm]
    print(f"angles over tolerance: {len(bad)}/{len(rows)}", [round(r['angle_deg'], 1) for r in bad[:8]])

if __name__ == "__main__":
    main()
