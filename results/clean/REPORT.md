# QC report — PASS

Injected defects: none

| stage | check | ok | detail |
|---|---|---|---|
| bpy | parts_found | ✅ | {"value": ["body", "lid"]} |
| bpy | scale_after_fix | ✅ | {"axis": "height_z", "measured_m": 0.26999998092651367, "spec_m": 0.27} |
| usd | stage_metadata | ✅ | {"upAxis": "Z", "metersPerUnit": 1.0} |
| usd | usd_validation | ✅ | {"checker": "UsdValidation (28 validators)"} |
| mjcf | collision_proxy_fill | ✅ | {"parts_over_limit": [], "limit": 4.0, "note": "sum of hull volumes / material volume (closed mesh volume, or area x wall thickness / 2 for open shells); far ab |
| mjcf | joint_sweep | ✅ | {"joint": "lid_hinge", "range_deg": [-110.0, 0.0], "steps": 45, "filterparent": "disabled", "worst_penetration_mm": 0.0, "tolerance_mm": 3.0, "note": "MuJoCo pa |
