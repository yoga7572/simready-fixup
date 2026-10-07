# QC report — FAIL

Injected defects: none

| stage | check | ok | detail |
|---|---|---|---|
| bpy | parts_found | ✅ | {"value": ["geometry_0"]} |
| bpy | scale_after_fix | ✅ | {"axis": "width_x", "measured_m": 0.3999999910593033, "spec_m": 0.4} |
| usd | stage_metadata | ✅ | {"upAxis": "Z", "metersPerUnit": 1.0} |
| usd | usd_validation | ✅ | {"checker": "UsdValidation (28 validators)"} |
| mjcf | collision_proxy_fill | ✅ | {"parts_over_limit": [], "limit": 4.0, "note": "sum of hull volumes / material volume (closed mesh volume, or area x wall thickness / 2 for open shells); far ab |
| mjcf | joint_sweep | ❌ | {"joint": "lid_hinge", "range_deg": [-30.0, 68.0], "steps": 45, "filterparent": "disabled", "worst_penetration_mm": 25.22129963537873, "tolerance_mm": 3.0, "not |
