#!/usr/bin/env bash
# One-command pipeline: GLB -> bpy fix-up -> USD physics -> CoACD/MJCF -> MuJoCo sweep -> report.
# usage: scripts/run_pipeline.sh <in.glb> <spec.json> <outdir> [--inject wrong-scale|lid-through-box|wrong-axis]
set -eo pipefail
GLB=$1; SPEC=$2; OUT=$3; INJ=${5:-}
cd "$(dirname "$0")/.."
B=(); U=(); M=()
case "$INJ" in wrong-scale) B=(--inject wrong-scale);; lid-through-box|wrong-axis) U=(--inject "$INJ"); M=(--inject "$INJ");; esac
uv run python -m simready_fixup.stage_bpy "$GLB" "$SPEC" "$OUT" "${B[@]}"
uv run python -m simready_fixup.stage_usd "$OUT" "$SPEC" "${U[@]}"
uv run python -m simready_fixup.stage_mjcf "$OUT" "$SPEC" "${M[@]}"
uv run python -m simready_fixup.report "$OUT"
MUJOCO_GL=glfw uv run python -m simready_fixup.visualize "$OUT"
