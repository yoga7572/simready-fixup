"""Stage 1 (Blender/bpy, headless): import GLB -> inspect -> fix units/origin/naming -> export USD + per-part OBJ.

Runs in its own process because bpy bundles its own USD build; stage 2 uses pip usd-core.
Usage: python -m simready_fixup.stage_bpy in.glb spec.json outdir [--inject wrong-scale]
"""
from __future__ import annotations
import argparse, json, os, sys
import numpy as np

def _mesh_arrays(obj):
    import bpy  # noqa
    me = obj.data
    me.calc_loop_triangles()
    v = np.empty(len(me.vertices) * 3, dtype=np.float64)
    me.vertices.foreach_get("co", v)
    v = v.reshape(-1, 3)
    mw = np.array(obj.matrix_world, dtype=np.float64)
    v_world = (mw @ np.c_[v, np.ones(len(v))].T).T[:, :3]
    f = np.array([t.vertices[:] for t in me.loop_triangles], dtype=np.int64)
    return v_world, f

def _bbox(objs):
    pts = np.vstack([_mesh_arrays(o)[0] for o in objs])
    return pts.min(0), pts.max(0)

def _remove_fragments(obj, min_faces):
    """Drop tiny disconnected shells (AI 'dust'). Rebuilds the mesh from the kept components; returns (kept, dropped)."""
    import bpy, trimesh
    v, f = _mesh_arrays(obj)
    tm = trimesh.Trimesh(v, f, process=False)
    comps = tm.split(only_watertight=False)
    keep = [c for c in comps if len(c.faces) >= min_faces]
    if len(keep) == len(comps): return len(comps), 0
    merged = trimesh.util.concatenate(keep) if len(keep) > 1 else keep[0]
    me = bpy.data.meshes.new(obj.data.name + "_clean")
    me.from_pydata(merged.vertices.tolist(), [], merged.faces.tolist()); me.update()
    old = obj.data; obj.data = me; obj.matrix_world.identity()
    bpy.data.meshes.remove(old)
    return len(keep), len(comps) - len(keep)

def _split_by_plane(obj, at_z, names):
    """Cut a fused mesh at the horizontal plane z=at_z into two closed parts (lower=names[0], upper=names[1]).
    AI generators usually fuse the lid into the body; the seam height is hand-annotated in the spec."""
    import bpy, bmesh
    me = obj.data
    bm = bmesh.new(); bm.from_mesh(me)
    bmesh.ops.bisect_plane(bm, geom=bm.verts[:] + bm.edges[:] + bm.faces[:], dist=1e-6,
                           plane_co=(0.0, 0.0, at_z), plane_no=(0.0, 0.0, 1.0), clear_inner=False, clear_outer=False)
    bm.faces.index_update()
    upper = {f.index for f in bm.faces if f.calc_center_median().z > at_z + 1e-7}
    out = {}
    for name, keep_upper in ((names[0], False), (names[1], True)):
        b2 = bm.copy(); b2.faces.ensure_lookup_table()
        drop = [f for f in b2.faces if (f.index in upper) != keep_upper]
        bmesh.ops.delete(b2, geom=drop, context="FACES")
        boundary = [e for e in b2.edges if e.is_boundary]
        if boundary:
            # cap the cut. triangle_fill respects nested loops: a wall with thickness yields an outer and an inner loop
            # and gets a ring cap, so a hollow body stays hollow (holes_fill would seal the opening and fill the box).
            try:
                bmesh.ops.triangle_fill(b2, use_beauty=True, use_dissolve=False, edges=boundary)
            except Exception:
                bmesh.ops.holes_fill(b2, edges=boundary, sides=0)
        new_me = bpy.data.meshes.new(name)
        for mat in me.materials: new_me.materials.append(mat)
        b2.to_mesh(new_me); b2.free()
        new_obj = bpy.data.objects.new(name, new_me); bpy.context.scene.collection.objects.link(new_obj)
        out[name] = new_obj
    bm.free()
    bpy.data.objects.remove(obj, do_unlink=True)
    return out

def _render_preview(objs, path):
    """Cycles CPU render of the fixed-up asset from a three-quarter view (inspection artifact, not a product render)."""
    import bpy, math, mathutils
    scene = bpy.context.scene
    lo, hi = _bbox(objs); c = (lo + hi) / 2; r = float(max(hi - lo))
    cam = bpy.data.objects.new("preview_cam", bpy.data.cameras.new("preview_cam")); scene.collection.objects.link(cam)
    cam.location = (c[0] + 2.0 * r, c[1] - 2.0 * r, c[2] + 1.4 * r)
    cam.rotation_euler = (mathutils.Vector(c) - cam.location).to_track_quat("-Z", "Y").to_euler()
    scene.camera = cam
    sun = bpy.data.lights.new("preview_sun", "SUN"); sun.energy = 3.0
    light = bpy.data.objects.new("preview_sun", sun); scene.collection.objects.link(light)
    light.rotation_euler = (math.radians(50), 0.0, math.radians(35))
    world = bpy.data.worlds.new("preview_world"); world.use_nodes = True
    bg = world.node_tree.nodes["Background"]; bg.inputs[0].default_value = (0.92, 0.92, 0.92, 1.0); bg.inputs[1].default_value = 1.0
    scene.world = world
    scene.render.engine = "CYCLES"; scene.cycles.samples = 24; scene.cycles.device = "CPU"
    scene.render.resolution_x = 720; scene.render.resolution_y = 540; scene.render.resolution_percentage = 100
    scene.render.image_settings.file_format = "PNG"; scene.render.filepath = path
    bpy.ops.render.render(write_still=True)
    bpy.data.objects.remove(cam, do_unlink=True); bpy.data.objects.remove(light, do_unlink=True)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("glb"); ap.add_argument("spec"); ap.add_argument("outdir")
    ap.add_argument("--inject", choices=["wrong-scale"], default=None)
    a = ap.parse_args()
    import bpy, trimesh
    spec = json.load(open(a.spec))
    os.makedirs(a.outdir, exist_ok=True)
    report = {"stage": "bpy", "blender": bpy.app.version_string, "input": os.path.abspath(a.glb), "checks": [], "actions": []}

    bpy.ops.wm.read_factory_settings(use_empty=True)
    bpy.ops.import_scene.gltf(filepath=a.glb)
    objs = [o for o in bpy.context.scene.objects if o.type == "MESH"]
    report["checks"].append({"id": "parts_found", "value": [o.name for o in objs], "ok": len(objs) >= 1})

    # apply transforms so vertex data is in world space
    for o in objs: o.select_set(True)
    bpy.context.view_layer.objects.active = objs[0]
    bpy.ops.object.transform_apply(location=True, rotation=True, scale=True)

    # loose fragments (AI generators leave thousands of tiny shells)
    min_faces = int(spec.get("min_component_faces", 0))
    if min_faces:
        for o in objs:
            kept, dropped = _remove_fragments(o, min_faces)
            if dropped: report["actions"].append({"id": "remove_fragments", "object": o.name, "kept_components": kept, "dropped_components": dropped, "min_faces": min_faces})

    # face budget: AI generators emit 500k+ triangles; sims want tens of thousands. Decimate (collapse) per object.
    budget = int(spec.get("face_budget", 60000)); total = sum(len(o.data.polygons) for o in objs)
    if total > budget:
        for o in objs:
            share = len(o.data.polygons) / total
            mod = o.modifiers.new("decimate", "DECIMATE"); mod.ratio = max(0.001, budget * share / max(1, len(o.data.polygons)))
            bpy.context.view_layer.objects.active = o
            bpy.ops.object.modifier_apply(modifier=mod.name)
        report["actions"].append({"id": "decimate", "faces_before": total, "faces_after": sum(len(o.data.polygons) for o in objs), "budget": budget})

    lo, hi = _bbox(objs); size = hi - lo
    report["checks"].append({"id": "bbox_raw_m", "min": lo.tolist(), "max": hi.tolist(), "size": size.tolist()})
    # unit / scale check against the spec (basis recorded in spec["dimension_basis"])
    want = spec["dimensions_m"]; axis_key = spec.get("scale_axis", "height_z"); ax = {"width_x": 0, "depth_y": 1, "height_z": 2}[axis_key]
    want_h = float(want[axis_key])
    ratio = want_h / float(size[ax])
    scale_ok = abs(ratio - 1.0) < 0.05
    report["checks"].append({"id": "scale_vs_spec_input", "axis": axis_key, "measured_m": float(size[ax]), "spec_m": want_h,
                             "ratio": ratio, "input_ok": scale_ok, "basis": spec.get("dimension_basis")})
    if not scale_ok and a.inject != "wrong-scale":
        for o in objs: o.scale = (ratio, ratio, ratio)
        bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)
        report["actions"].append({"id": "rescale", "factor": ratio, "reason": f"{axis_key} differs from spec by >5%"})
    elif a.inject == "wrong-scale":
        report["actions"].append({"id": "INJECT_wrong_scale", "note": "rescale deliberately skipped", "factor": ratio})
    lo, hi = _bbox(objs)
    ratio2 = want_h / float((hi - lo)[ax])
    report["checks"].append({"id": "scale_after_fix", "axis": axis_key, "measured_m": float((hi - lo)[ax]), "spec_m": want_h, "ok": abs(ratio2 - 1.0) < 0.05})
    # origin: base centre at world origin, Z-up (gltf importer already converted Y-up -> Blender Z-up)
    shift = np.array([-(lo[0] + hi[0]) / 2, -(lo[1] + hi[1]) / 2, -lo[2]])
    for o in objs: o.location = (o.location.x + shift[0], o.location.y + shift[1], o.location.z + shift[2])
    bpy.ops.object.transform_apply(location=True, rotation=False, scale=False)
    report["actions"].append({"id": "recenter_origin", "shift_m": shift.tolist(), "convention": "Z-up, metres, base centre at origin"})
    lo, hi = _bbox(objs)
    report["checks"].append({"id": "bbox_fixed_m", "min": lo.tolist(), "max": hi.tolist(), "size": (hi - lo).tolist(),
                             "up_axis": "Z", "meters_per_unit": 1.0})

    # fused AI mesh: cut body/lid at the hand-annotated seam height (spec["split"]) before assigning part roles
    split = spec.get("split")
    if split and len(objs) == 1:
        new_objs = _split_by_plane(objs[0], float(split["at_m"]), list(split["names"]))
        objs = list(new_objs.values())
        report["actions"].append({"id": "split_plane", "axis": "z", "at_m": split["at_m"], "names": split["names"], "basis": split.get("basis")})
    elif split and len(objs) > 1:
        report["actions"].append({"id": "split_plane_skipped", "reason": f"{len(objs)} parts already present"})

    # part roles: thinnest-in-Z & highest part = lid, largest volume = body (heuristic; spec may override by name)
    parts = {}
    for o in objs:
        v, f = _mesh_arrays(o)
        tm = trimesh.Trimesh(v, f, process=False)
        parts[o.name] = {"obj": o, "tm": tm, "bbox": (v.min(0), v.max(0))}
    names = list(parts)
    if not all(n in spec["parts"] for n in names):
        ranked = sorted(names, key=lambda n: (parts[n]["bbox"][1][2] - parts[n]["bbox"][0][2]))
        rename = {ranked[-1]: "body"} if len(ranked) == 1 else {ranked[0]: "lid", ranked[-1]: "body"}
        for old, new in rename.items():
            parts[old]["obj"].name = new; parts[new] = parts.pop(old)
        report["actions"].append({"id": "rename_parts", "map": rename, "rule": "thinnest Z extent -> lid, else body"})

    # export per-part OBJ (Z-up, metres) for the MJCF/CoACD stage, plus mesh QC
    part_info = {}
    for name, p in parts.items():
        tm = p["tm"]
        path = os.path.join(a.outdir, f"{name}.obj"); tm.export(path)
        part_info[name] = {"obj": path, "faces": int(len(tm.faces)), "watertight": bool(tm.is_watertight),
                           "bbox_min": tm.bounds[0].tolist(), "bbox_max": tm.bounds[1].tolist()}
    report["parts"] = part_info

    # inspection render (Cycles CPU); failure here must not fail the stage
    try:
        _render_preview(list(parts[n]["obj"] for n in parts), os.path.join(a.outdir, "preview.png"))
        report["preview"] = os.path.join(a.outdir, "preview.png")
    except Exception as ex:  # pragma: no cover
        report["preview_error"] = repr(ex)[:200]

    # USD export via Blender (Z-up, metersPerUnit=1). Physics schemas are authored in stage 2.
    usd_path = os.path.join(a.outdir, "asset_visual.usda")
    kwargs = dict(filepath=usd_path, export_materials=True, selected_objects_only=False)
    try:
        bpy.ops.wm.usd_export(**kwargs, root_prim_path="/asset")
    except TypeError:
        bpy.ops.wm.usd_export(**kwargs)
    report["usd_visual"] = usd_path
    json.dump(report, open(os.path.join(a.outdir, "report_bpy.json"), "w"), indent=2)
    print(json.dumps({k: report[k] for k in ("checks", "actions", "parts")}, indent=1))

if __name__ == "__main__":
    main()
