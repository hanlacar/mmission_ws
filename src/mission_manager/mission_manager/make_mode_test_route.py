#!/usr/bin/env python3
import argparse
from pathlib import Path
import pandas as pd
import numpy as np
import yaml

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("input_csv")
    ap.add_argument("input_yaml")
    ap.add_argument("--start-mode", type=int, required=True)
    ap.add_argument("--end-mode", type=int, required=True)
    ap.add_argument("--base-segment", default="AAA_BASE")
    ap.add_argument("--output", required=True, help="output csv path")
    args = ap.parse_args()

    if not (1 <= args.start_mode <= args.end_mode <= 11):
        raise SystemExit("mode range must satisfy 1 <= start <= end <= 11")

    df = pd.read_csv(args.input_csv)
    with open(args.input_yaml, encoding="utf-8") as f:
        meta = yaml.safe_load(f)

    if "segment_id" not in df.columns:
        raise SystemExit("input CSV requires segment_id")
    if args.base_segment not in set(df["segment_id"].astype(str)):
        raise SystemExit(f"base segment not found: {args.base_segment}")

    base = df[df["segment_id"].astype(str) == args.base_segment].copy()
    base = base.sort_values("point_index" if "point_index" in base.columns else base.index.name)

    route = base[(base["mode"] >= args.start_mode) & (base["mode"] <= args.end_mode)].copy()
    if route.empty:
        raise SystemExit("no points in requested mode range")

    xy = route[["x_m", "y_m"]].to_numpy(float)
    keep = np.ones(len(route), dtype=bool)
    if len(route) > 1:
        keep[1:] = np.linalg.norm(np.diff(xy, axis=0), axis=1) > 1e-6
    route = route.iloc[np.where(keep)[0]].copy()

    route["index"] = range(len(route))
    if "event" not in route.columns:
        route["event"] = "NONE"

    cols = [
        "index","latitude","longitude","x_m","y_m",
        "direction","mode","drive_level","event"
    ]
    out_csv = Path(args.output)
    out_yaml = out_csv.with_suffix(".yaml")
    route[cols].to_csv(out_csv, index=False)

    out_meta = {
        "format_version": 1,
        "origin_lat": float(meta["origin_lat"]),
        "origin_lon": float(meta["origin_lon"]),
        "loop": False,
        "created_at": f"mode_subset_{args.start_mode}_{args.end_mode}",
    }
    with open(out_yaml, "w", encoding="utf-8") as f:
        yaml.safe_dump(out_meta, f, sort_keys=False, allow_unicode=True)

    jumps = np.linalg.norm(np.diff(route[["x_m","y_m"]].to_numpy(float), axis=0), axis=1)
    print(f"saved: {out_csv}")
    print(f"saved: {out_yaml}")
    print(f"points: {len(route)}")
    print(f"max consecutive jump: {jumps.max() if len(jumps) else 0.0:.3f} m")

if __name__ == "__main__":
    main()

