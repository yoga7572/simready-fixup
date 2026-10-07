"""Generate a synthetic 'AI-generated-like' lidded box GLB for pipeline development.

The asset deliberately ships with typical defects: wrong unit scale (x100, as if centimetres were
metres) so the fix-up stage has real work to do. Geometry is authored Z-up and converted to the
glTF Y-up convention on export.
"""
from __future__ import annotations
import argparse, json
import numpy as np
import trimesh

def _box(extents, center):
    m = trimesh.creation.box(extents=extents)
    m.apply_translation(center)
    return m

def make_lidded_box(w=0.40, d=0.30, h=0.25, wall=0.01, lid_t=0.02):
    """Hollow open-top box (5 slabs) + flat lid resting on the rim. Units: metres, Z-up, origin at base centre."""
    slabs = [
        _box((w, d, wall), (0, 0, wall / 2)),                                  # bottom
        _box((w, wall, h), (0, -d / 2 + wall / 2, h / 2)),                     # front wall
        _box((w, wall, h), (0, d / 2 - wall / 2, h / 2)),                      # back wall
        _box((wall, d, h), (-w / 2 + wall / 2, 0, h / 2)),                     # left wall
        _box((wall, d, h), (w / 2 - wall / 2, 0, h / 2)),                      # right wall
    ]
    body = trimesh.util.concatenate(slabs)
    lid = _box((w, d, lid_t), (0, 0, h + lid_t / 2))
    return body, lid

Z_UP_TO_Y_UP = np.array([[1, 0, 0, 0], [0, 0, 1, 0], [0, -1, 0, 0], [0, 0, 0, 1]], dtype=float)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("out_glb")
    ap.add_argument("--spec", required=True, help="where to write the asset spec json")
    ap.add_argument("--scale-defect", type=float, default=100.0, help="multiply geometry to mimic wrong units")
    ap.add_argument("--fused", action="store_true", help="export body+lid as ONE mesh, like most AI generators do")
    a = ap.parse_args()
    body, lid = make_lidded_box()
    scene = trimesh.Scene()
    pieces = (("box", trimesh.util.concatenate([body, lid])),) if a.fused else (("body", body), ("lid", lid))
    for name, m in pieces:
        m = m.copy()
        m.apply_scale(a.scale_defect)
        m.apply_transform(Z_UP_TO_Y_UP)
        scene.add_geometry(m, node_name=name, geom_name=name)
    scene.export(a.out_glb)
    spec = {
        "name": "storage_box_with_lid",
        "description": "Plastic storage box with a hinged lid (synthetic dev asset)",
        "dimensions_m": {"width_x": 0.40, "depth_y": 0.30, "height_z": 0.27},
        "dimension_basis": "synthetic spec; for AI assets use the prompt/datasheet dimensions",
        "density_kg_m3": 950.0,
        "density_basis": "polypropylene, assumed uniform",
        "parts": {"body": {"role": "base"}, "lid": {"role": "moving"}},
        "split": {"axis": "z", "at_m": 0.25, "names": ["body", "lid"], "basis": "hand-annotated seam height between body rim and lid"},
        "joints": [{
            "name": "lid_hinge", "type": "revolute", "parent": "body", "child": "lid",
            "axis": [1, 0, 0], "origin_m": [0.0, 0.15, 0.25],
            "limits_deg": [-110.0, 0.0],
            "basis": "hand-annotated: hinge along the back top edge of the body; 0 = closed, negative = opening (right-hand rule about +X)"
        }]
    }
    json.dump(spec, open(a.spec, "w"), indent=2)
    print("wrote", a.out_glb, "and", a.spec)

if __name__ == "__main__":
    main()
