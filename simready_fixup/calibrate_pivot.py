"""Pivot calibration: re-sweep the hinge over a small grid of candidate origins using the hulls already produced by
stage 3, and report the worst penetration for each. Tells a hand-annotated pivot error apart from a genuine
lid/body interference (a genuine one stays bad for every candidate).

Usage: python -m simready_fixup.calibrate_pivot outdir spec.json --dy -0.02,0,0.02 --dz -0.02,0,0.02
"""
import argparse, glob, json, os, tempfile
import numpy as np, trimesh, mujoco

def main():
    ap = argparse.ArgumentParser(); ap.add_argument("outdir"); ap.add_argument("spec")
    ap.add_argument("--dy", default="-0.02,0,0.02"); ap.add_argument("--dz", default="-0.02,0,0.02")
    ap.add_argument("--steps", type=int, default=30)
    ap.add_argument("--lid-scale", default="1.0", help="uniform scale of the child part about the hinge, e.g. 1.0,0.97,0.94")
    ap.add_argument("--limits", default=None, help="override joint limits, e.g. -20,68")
    a = ap.parse_args()
    spec = json.load(open(a.spec)); j = spec["joints"][0]
    origin0 = np.array(j["origin_m"], float); axis = j["axis"]; lo, hi = j["limits_deg"]
    if a.limits: lo, hi = [float(x) for x in a.limits.split(",")]
    meshdir = os.path.join(a.outdir, "mjcf_meshes")
    body_hulls = sorted(os.path.basename(p) for p in glob.glob(os.path.join(meshdir, f"{j['parent']}_hull*.obj")))
    lid_hulls = sorted(os.path.basename(p) for p in glob.glob(os.path.join(meshdir, f"{j['child']}_hull*.obj")))
    results = []
    for ls in [float(x) for x in a.lid_scale.split(",")]:
      for dy in [float(x) for x in a.dy.split(",")]:
        for dz in [float(x) for x in a.dz.split(",")]:
            origin = origin0 + np.array([0, dy, dz])
            tmp = tempfile.mkdtemp(prefix="pivot_")
            # lid hulls are stored relative to origin0; re-express them relative to the candidate origin
            assets = []
            for f in body_hulls:
                assets.append((f[:-4], os.path.join(meshdir, f)))
            for f in lid_hulls:
                t = trimesh.load(os.path.join(meshdir, f), force="mesh"); t.apply_scale(ls); t.apply_translation(origin0 - origin)
                p = os.path.join(tmp, f); t.export(p); assets.append((f[:-4], p))
            xml = f"""<mujoco><compiler angle="degree"/><option><flag filterparent="disable"/></option>
<asset>{''.join(f'<mesh name="{n}" file="{p}"/>' for n, p in assets)}</asset>
<worldbody><body name="body">{''.join(f'<geom name="{f[:-4]}" type="mesh" mesh="{f[:-4]}"/>' for f in body_hulls)}
<body name="lid" pos="{origin[0]} {origin[1]} {origin[2]}"><joint name="h" type="hinge" axis="{axis[0]} {axis[1]} {axis[2]}" range="{lo} {hi}" limited="true"/>
{''.join(f'<geom name="{f[:-4]}" type="mesh" mesh="{f[:-4]}"/>' for f in lid_hulls)}</body></body></worldbody></mujoco>"""
            m = mujoco.MjModel.from_xml_string(xml); d = mujoco.MjData(m); q = m.jnt_qposadr[0]
            worst = 0.0; worst_angle = None; over = 0
            for deg in np.linspace(lo, hi, a.steps):
                d.qpos[q] = np.deg2rad(deg); mujoco.mj_forward(m, d)
                pen = max([-d.contact[i].dist for i in range(d.ncon) if d.contact[i].dist < 0] + [0.0])
                if pen > worst: worst, worst_angle = pen, deg
                if pen > 0.003: over += 1
            results.append((dy, dz, worst * 1000, worst_angle, over, ls))
            print(f"lid_scale={ls:.3f} dy={dy:+.3f} dz={dz:+.3f}  worst {worst*1000:6.1f} mm at {worst_angle if worst_angle is None else round(worst_angle,1)} deg  angles>3mm {over}/{a.steps}", flush=True)
    best = min(results, key=lambda r: r[2])
    print("best candidate:", best)
    json.dump([dict(dy=r[0], dz=r[1], worst_mm=r[2], worst_angle=r[3], over=r[4], lid_scale=r[5]) for r in results], open(os.path.join(a.outdir, "pivot_calibration.json"), "w"), indent=1)

if __name__ == "__main__":
    main()
