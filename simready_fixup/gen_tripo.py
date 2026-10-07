"""Generate a GLB with the Tripo API (text-to-model). Needs TRIPO_API_KEY in the environment or .env.

Usage: python -m simready_fixup.gen_tripo "a plastic storage box with a hinged lid, 40x30x27 cm" out.glb
The spec (dimensions, density, joints) is still written by hand — the generator does not know real-world scale.
"""
from __future__ import annotations
import os, sys, time, json, urllib.request

API = "https://api.tripo3d.ai/v2/openapi"

def _req(method, path, key, body=None):
    data = json.dumps(body).encode() if body is not None else None
    req = urllib.request.Request(API + path, data=data, method=method,
                                 headers={"Authorization": f"Bearer {key}", "Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=60) as r:
        return json.load(r)

def main():
    prompt, out = sys.argv[1], sys.argv[2]
    key = os.environ.get("TRIPO_API_KEY")
    if not key and os.path.exists(".env"):
        for line in open(".env"):
            if line.startswith("TRIPO_API_KEY="): key = line.split("=", 1)[1].strip()
    if not key: sys.exit("TRIPO_API_KEY not set (put it in .env, which is git-ignored)")
    task = _req("POST", "/task", key, {"type": "text_to_model", "prompt": prompt})
    tid = task["data"]["task_id"]; print("task", tid)
    while True:
        st = _req("GET", f"/task/{tid}", key)["data"]
        print(" status:", st.get("status"), st.get("progress"))
        if st.get("status") in ("success", "failed", "cancelled", "unknown"): break
        time.sleep(5)
    if st.get("status") != "success": sys.exit(f"generation failed: {st}")
    url = st["output"].get("pbr_model") or st["output"].get("model")
    urllib.request.urlretrieve(url, out); print("saved", out)
    json.dump({"task_id": tid, "prompt": prompt, "output": st["output"]}, open(out + ".tripo.json", "w"), indent=2)

if __name__ == "__main__":
    main()
