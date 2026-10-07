# QC report — FAIL

Injected defects: ['INJECT_wrong_axis']

| stage | check | ok | detail |
|---|---|---|---|
| bpy | parts_found | ✅ | {"value": ["body", "lid"]} |
| bpy | scale_after_fix | ✅ | {"measured_height_m": 0.26999998092651367, "spec_height_m": 0.27} |
| usd | stage_metadata | ✅ | {"upAxis": "Z", "metersPerUnit": 1.0} |
| usd | usd_validation | ✅ | {"checker": "UsdValidation (28 validators)"} |
| mjcf | joint_sweep | ❌ | {"joint": "lid_hinge", "range_deg": [-110.0, 0.0], "steps": 45, "filterparent": "disabled", "worst_penetration_mm": 19.024355070214696, "tolerance_mm": 3.0, "no |
