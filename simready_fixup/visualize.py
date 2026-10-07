"""Plot the joint sweep (angle vs. max penetration) and render a validation video of the sweep with MuJoCo.

Usage: python -m simready_fixup.visualize outdir [--no-video]
"""
import argparse, json, os, subprocess, shutil
import numpy as np

def plot(outdir, rep):
    import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
    rows = rep["sweep"]; chk = rep["checks"][0]
    a = [r["angle_deg"] for r in rows]; p = [r["max_penetration_mm"] for r in rows]
    fig, ax = plt.subplots(figsize=(7, 3.2))
    ax.plot(a, p, marker=".", lw=1.2, label="max penetration (mm)")
    ax.axhline(chk["tolerance_mm"], color="r", ls="--", lw=1, label=f"tolerance {chk['tolerance_mm']} mm")
    ax.set_xlabel(f"{chk['joint']} angle (deg)"); ax.set_ylabel("mm"); ax.set_yscale("symlog", linthresh=1)
    ax.set_title(f"joint sweep — {'PASS' if chk['ok'] else 'FAIL'} (worst {chk['worst_penetration_mm']:.1f} mm, filterparent disabled)")
    ax.legend(loc="upper right", fontsize=8); fig.tight_layout()
    path = os.path.join(outdir, "sweep.png"); fig.savefig(path, dpi=130); plt.close(fig); return path

def video(outdir, rep, w=960, h=540):
    import mujoco
    m = mujoco.MjModel.from_xml_path(rep["mjcf"]); d = mujoco.MjData(m)
    jid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, rep["checks"][0]["joint"]); qadr = m.jnt_qposadr[jid]
    r = mujoco.Renderer(m, height=h, width=w)
    cam = mujoco.MjvCamera(); cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    cam.lookat[:] = m.stat.center; cam.distance = 1.5 * m.stat.extent; cam.azimuth = 135; cam.elevation = -22
    m.vis.scale.contactwidth = 0.08; m.vis.scale.contactheight = 0.02  # contact markers relative to model extent
    opt = mujoco.MjvOption(); opt.geomgroup[:] = 0; opt.geomgroup[1] = 1; opt.geomgroup[3] = 1  # visual + hulls
    opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = 1; opt.flags[mujoco.mjtVisFlag.mjVIS_TRANSPARENT] = 1
    frames_dir = os.path.join(outdir, "frames"); shutil.rmtree(frames_dir, ignore_errors=True); os.makedirs(frames_dir)
    rows = rep["sweep"]; seq = rows + rows[::-1]
    from PIL import Image, ImageDraw, ImageFont
    try: font = ImageFont.truetype("/System/Library/Fonts/Helvetica.ttc", 24)
    except Exception: font = ImageFont.load_default()
    verdict = "PASS" if rep["checks"][0]["ok"] else "FAIL"; tol = rep["checks"][0]["tolerance_mm"]
    for i, row in enumerate(seq):
        d.qpos[qadr] = np.deg2rad(row["angle_deg"]); mujoco.mj_forward(m, d)
        r.update_scene(d, camera=cam, scene_option=opt); img = Image.fromarray(r.render())
        dr = ImageDraw.Draw(img)
        txt = f"{os.path.basename(outdir)}  |  {rep['checks'][0]['joint']} {row['angle_deg']:.1f} deg  |  penetration {row['max_penetration_mm']:.1f} mm (tol {tol})  |  {verdict}"
        dr.rectangle((10, 10, 10 + dr.textlength(txt, font=font) + 20, 50), fill=(0, 0, 0, 160)); dr.text((20, 18), txt, fill=(255, 80, 80) if row["max_penetration_mm"] > tol else (255, 255, 255), font=font)
        img.save(os.path.join(frames_dir, f"f{i:04d}.png"))
    r.close()
    out = os.path.join(outdir, "sweep.mp4")
    subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-framerate", "15", "-i", os.path.join(frames_dir, "f%04d.png"),
                    "-pix_fmt", "yuv420p", "-crf", "22", out], check=True)
    shutil.rmtree(frames_dir); return out

def main():
    ap = argparse.ArgumentParser(); ap.add_argument("outdir"); ap.add_argument("--no-video", action="store_true"); a = ap.parse_args()
    rep = json.load(open(os.path.join(a.outdir, "report_mjcf.json")))
    print("plot:", plot(a.outdir, rep))
    if not a.no_video:
        try: print("video:", video(a.outdir, rep))
        except Exception as e: print("video skipped:", repr(e)[:300])

if __name__ == "__main__":
    main()
