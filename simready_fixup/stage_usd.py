"""Stage 2 (pip usd-core): author UsdPhysics on the Blender-exported USD and run the compliance checker.

Usage: python -m simready_fixup.stage_usd outdir spec.json [--inject lid-through-box|wrong-axis]
"""
from __future__ import annotations
import argparse, json, os
import numpy as np
import trimesh
from pxr import Usd, UsdGeom, UsdPhysics, Gf, Sdf

def mass_props(obj_path, density, wall_thickness=0.003):
    """Mass, COM and inertia from the watertight connected components of the part (hollow parts stay hollow).
    Falls back to the convex hull only when no component is closed; the basis is recorded in the report."""
    tm = trimesh.load(obj_path, force="mesh")
    comps = []
    for c in tm.split(only_watertight=False):
        if not c.is_watertight:
            try: trimesh.repair.fill_holes(c)
            except Exception: pass
        if c.is_watertight and c.volume > 0: comps.append(c)
    if comps:
        basis = f"sum of {len(comps)} watertight component(s)"
        masses = np.array([c.volume * density for c in comps]); vol = float(sum(c.volume for c in comps))
        coms = np.array([c.center_mass for c in comps]); mass = float(masses.sum())
        com = (masses[:, None] * coms).sum(0) / mass
        I = np.zeros((3, 3))
        for c, mc, cc in zip(comps, masses, coms):
            d = cc - com
            I += c.moment_inertia * density + mc * (np.dot(d, d) * np.eye(3) - np.outer(d, d))  # parallel-axis shift to the part COM
    else:
        # open shell: treat it as a thin wall of assumed thickness instead of filling it (a convex hull would make a
        # hollow box 10x too heavy). Inertia from the surface mass distribution.
        t = wall_thickness; area = float(tm.area)
        vol = area * t / 2.0  # the mesh usually has inner+outer skins, so half the area is the outer wall
        mass = vol * density
        tri = tm.triangles; a = tm.area_faces; cen = tri.mean(axis=1)
        com = (a[:, None] * cen).sum(0) / a.sum()
        d = cen - com; mi = mass * a / a.sum()
        I = np.zeros((3, 3))
        for di, m_i in zip(d, mi): I += m_i * (np.dot(di, di) * np.eye(3) - np.outer(di, di))
        basis = f"open shell: surface area {area:.3f} m2 x assumed wall thickness {t*1000:.1f} mm / 2 (no closed component after hole filling)"
    w, V = np.linalg.eigh(I)
    if np.linalg.det(V) < 0: V[:, 0] *= -1
    q = trimesh.transformations.quaternion_from_matrix(np.r_[np.c_[V, [0, 0, 0]], [[0, 0, 0, 1]]])  # (w,x,y,z)
    return {"mass_kg": mass, "volume_m3": vol, "com": com.tolist(), "diag_inertia": w.tolist(), "principal_quat_wxyz": q.tolist(), "basis": basis}

def validate_stage(stage, part_paths):
    """USD validation: the UsdValidation framework (all registered validators) plus built-in physics sanity checks."""
    res = {"id": "usd_validation", "errors": [], "warnings": [], "checker": None}
    try:
        from pxr import UsdValidation
        reg = UsdValidation.ValidationRegistry(); vals = reg.GetOrLoadAllValidators()
        ctx = UsdValidation.ValidationContext(vals)
        for e in ctx.Validate(stage):
            if e.GetType() == UsdValidation.ValidationErrorType.Info: continue
            kind = "warnings" if e.GetType() == UsdValidation.ValidationErrorType.Warn else "errors"
            res[kind].append(f"{e.GetName()}: {e.GetMessage()}")
        res["checker"] = f"UsdValidation ({len(vals)} validators)"
    except Exception as ex:  # pragma: no cover
        res["checker"] = f"UsdValidation unavailable: {ex}"; res["errors"].append("validation framework unavailable")
    # built-in physics sanity checks (what a simulator actually needs)
    if UsdGeom.GetStageUpAxis(stage) != "Z": res["errors"].append("upAxis is not Z")
    if abs(UsdGeom.GetStageMetersPerUnit(stage) - 1.0) > 1e-9: res["errors"].append("metersPerUnit != 1")
    for name, path in part_paths.items():
        prim = stage.GetPrimAtPath(path)
        if not prim.HasAPI(UsdPhysics.RigidBodyAPI): res["errors"].append(f"{name}: missing RigidBodyAPI")
        m = UsdPhysics.MassAPI(prim).GetMassAttr().Get() if prim.HasAPI(UsdPhysics.MassAPI) else None
        if not m or m <= 0: res["errors"].append(f"{name}: mass missing or <= 0")
        if not any(c.HasAPI(UsdPhysics.CollisionAPI) for c in Usd.PrimRange(prim)): res["errors"].append(f"{name}: no CollisionAPI under part")
    for prim in stage.Traverse():
        if prim.IsA(UsdPhysics.RevoluteJoint):
            j = UsdPhysics.RevoluteJoint(prim)
            for rel in (j.GetBody0Rel(), j.GetBody1Rel()):
                t = rel.GetTargets()
                if not t or not stage.GetPrimAtPath(t[0]).IsValid(): res["errors"].append(f"{prim.GetName()}: joint body target invalid")
            lo, hi = j.GetLowerLimitAttr().Get(), j.GetUpperLimitAttr().Get()
            if lo is None or hi is None or lo >= hi: res["errors"].append(f"{prim.GetName()}: bad limits {lo},{hi}")
    res["ok"] = not res["errors"]
    return res

def find_part_prims(stage, name):
    """Return (xform_prim, mesh_prim) for a Blender-exported object called `name`."""
    for prim in stage.Traverse():
        if prim.IsA(UsdGeom.Mesh) and (prim.GetName() == name or prim.GetParent().GetName() == name):
            xf = prim.GetParent()
            return xf, prim
    raise SystemExit(f"part prim not found: {name}")

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("outdir"); ap.add_argument("spec")
    ap.add_argument("--inject", choices=["lid-through-box", "wrong-axis"], default=None)
    a = ap.parse_args()
    spec = json.load(open(a.spec))
    src = os.path.join(a.outdir, "asset_visual.usda")
    stage = Usd.Stage.Open(src)
    rep = {"stage": "usd", "source": src, "checks": [], "actions": []}
    up = UsdGeom.GetStageUpAxis(stage); mpu = UsdGeom.GetStageMetersPerUnit(stage)
    rep["checks"].append({"id": "stage_metadata", "upAxis": str(up), "metersPerUnit": mpu, "ok": up == "Z" and abs(mpu - 1.0) < 1e-9})
    if up != "Z": UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z); rep["actions"].append({"id": "set_up_axis_Z"})
    if abs(mpu - 1.0) > 1e-9: UsdGeom.SetStageMetersPerUnit(stage, 1.0); rep["actions"].append({"id": "set_meters_per_unit_1"})

    scene = UsdPhysics.Scene.Define(stage, "/physicsScene")
    scene.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1)); scene.CreateGravityMagnitudeAttr(9.81)

    root = stage.GetPrimAtPath("/asset")
    if not root.IsValid():
        root = stage.GetDefaultPrim() or [p for p in stage.GetPseudoRoot().GetChildren() if p.GetName() != "physicsScene"][0]
    UsdPhysics.ArticulationRootAPI.Apply(root)
    stage.SetDefaultPrim(root)
    rep["actions"].append({"id": "articulation_root", "prim": str(root.GetPath())})

    part_paths = {}
    for name, pinfo in spec["parts"].items():
        xf, mesh = find_part_prims(stage, name)
        part_paths[name] = str(xf.GetPath())
        UsdPhysics.RigidBodyAPI.Apply(xf.GetPrim())
        UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
        mc = UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim())
        mc.CreateApproximationAttr(UsdPhysics.Tokens.convexDecomposition)
        mp = mass_props(os.path.join(a.outdir, f"{name}.obj"), spec["density_kg_m3"], float(spec.get("wall_thickness_m", 0.003)))
        massapi = UsdPhysics.MassAPI.Apply(xf.GetPrim())
        massapi.CreateMassAttr(float(mp["mass_kg"]))
        massapi.CreateCenterOfMassAttr(Gf.Vec3f(*mp["com"]))
        massapi.CreateDiagonalInertiaAttr(Gf.Vec3f(*mp["diag_inertia"]))
        qw, qx, qy, qz = mp["principal_quat_wxyz"]
        massapi.CreatePrincipalAxesAttr(Gf.Quatf(qw, qx, qy, qz))
        rep["actions"].append({"id": "physics_body", "part": name, "prim": part_paths[name], **mp, "density_basis": spec.get("density_basis")})

    for j in spec["joints"]:
        origin = np.array(j["origin_m"], dtype=float); axis = list(j["axis"])
        if a.inject == "lid-through-box": origin = origin + np.array([0, -0.05, 0]); rep["actions"].append({"id": "INJECT_lid_through_box", "origin_shift_m": [0, -0.05, 0]})
        if a.inject == "wrong-axis": axis = [0, 1, 0]; rep["actions"].append({"id": "INJECT_wrong_axis", "axis": axis})
        jp = Sdf.Path(part_paths[j["child"]]).GetParentPath().AppendChild(j["name"])
        joint = UsdPhysics.RevoluteJoint.Define(stage, jp)
        joint.CreateBody0Rel().SetTargets([Sdf.Path(part_paths[j["parent"]])])
        joint.CreateBody1Rel().SetTargets([Sdf.Path(part_paths[j["child"]])])
        ax = {(1, 0, 0): "X", (0, 1, 0): "Y", (0, 0, 1): "Z"}[tuple(int(x) for x in axis)]
        joint.CreateAxisAttr(ax)
        # joint frame expressed in each body's local frame; both part xforms sit at the asset origin after fix-up
        joint.CreateLocalPos0Attr(Gf.Vec3f(*origin)); joint.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
        joint.CreateLocalPos1Attr(Gf.Vec3f(*origin)); joint.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
        joint.CreateLowerLimitAttr(float(j["limits_deg"][0])); joint.CreateUpperLimitAttr(float(j["limits_deg"][1]))
        rep["actions"].append({"id": "revolute_joint", "prim": str(jp), "axis": ax, "origin_m": origin.tolist(), "limits_deg": j["limits_deg"], "basis": j.get("basis"),
                               "frame_note": "localPos0/1 are in body0/body1 local frames (identical to asset frame here because part xforms are identity)"})

    out = os.path.join(a.outdir, "asset_physics.usda")
    stage.GetRootLayer().Export(out)
    rep["usd_physics"] = out
    rep["checks"].append(validate_stage(Usd.Stage.Open(out), part_paths))
    json.dump(rep, open(os.path.join(a.outdir, "report_usd.json"), "w"), indent=2)
    print(json.dumps({"checks": rep["checks"], "joints": [x for x in rep["actions"] if x["id"] == "revolute_joint"]}, indent=1))

if __name__ == "__main__":
    main()
