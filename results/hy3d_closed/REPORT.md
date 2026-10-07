# QC report — FAIL

Injected defects: none

| stage | check | ok | detail |
|---|---|---|---|
| bpy | parts_found | ✅ | {"value": ["geometry_0"]} |
| bpy | scale_after_fix | ✅ | {"axis": "width_x", "measured_m": 0.4000000059604645, "spec_m": 0.4} |
| usd | stage_metadata | ✅ | {"upAxis": "Z", "metersPerUnit": 1.0} |
| usd | usd_validation | ✅ | {"checker": "UsdValidation (28 validators)"} |
| mjcf | collision_proxy_fill | ❌ | {"parts_over_limit": [["body", 35.6], ["lid", 16.2]], "limit": 4.0, "note": "sum of hull volumes / material volume (closed mesh volume, or area x wall thickness |
| mjcf | joint_sweep | ✅ | {"joint": "lid_hinge", "range_deg": [-110.0, 0.0], "steps": 45, "filterparent": "disabled", "worst_penetration_mm": 2.6668770297070163, "tolerance_mm": 3.0, "no |
