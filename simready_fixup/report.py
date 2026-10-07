"""Merge stage reports into report.json + REPORT.md with a single PASS/FAIL verdict."""
import json, os, sys
outdir = sys.argv[1]
reps = {k: json.load(open(os.path.join(outdir, f"report_{k}.json"))) for k in ("bpy", "usd", "mjcf")}
checks = [dict(stage=k, **c) for k, r in reps.items() for c in r["checks"] if "ok" in c]
verdict = "PASS" if all(c["ok"] for c in checks) else "FAIL"
injected = sorted({x["id"] for r in reps.values() for x in r.get("actions", []) if x["id"].startswith("INJECT")})
json.dump({"verdict": verdict, "injected_defects": injected, "checks": checks, "stages": reps}, open(os.path.join(outdir, "report.json"), "w"), indent=2)
lines = [f"# QC report — {verdict}", "", f"Injected defects: {injected or 'none'}", "", "| stage | check | ok | detail |", "|---|---|---|---|"]
for c in checks:
    detail = {k: v for k, v in c.items() if k not in ("stage", "id", "ok", "errors", "failed", "warnings")}
    lines.append(f"| {c['stage']} | {c['id']} | {'✅' if c['ok'] else '❌'} | {json.dumps(detail, ensure_ascii=False)[:160]} |")
open(os.path.join(outdir, "REPORT.md"), "w").write("\n".join(lines) + "\n")
print("\n".join(lines))
