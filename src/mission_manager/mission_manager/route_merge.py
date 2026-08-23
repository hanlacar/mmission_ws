"""
route_merge — 각각 따로 기록한 GPS 모범경로 CSV들을 하나로 이어붙인다.

■ 왜 필요한가
  구간을 따로따로 기록(출발+경사로, S코스, T주차, ...)한 뒤 순서대로
  이어 붙여 하나의 연속 경로로 만들고 싶을 때 쓴다. 단순히 파일을
  붙이면 두 가지가 깨진다:
    1) 각 CSV는 자기 기록 시작점을 origin(기준 위경도)으로 삼는다.
       origin이 다르면 x_m/y_m 좌표계가 서로 안 맞는다.
       → 첫 CSV의 origin으로 통일해 모든 점의 x_m/y_m을 재계산한다.
    2) index는 0부터 연속이어야 하고, route_loader가 이음새 간격이
       너무 크면(max_segment_m 초과) 거부한다.
       → index를 다시 매기고, 이음새 간격을 검사해 경고한다.

  direction(1/-1), mode, drive_level 컬럼은 각 점의 값을 그대로 보존한다.
  즉 A(전진)→B(후진)를 이어 붙이면 이음새가 자동으로 cusp가 되어
  follower가 정지→방향전환→주행한다.

■ 사용법 (colcon build 후)
    ros2 run mission_manager route_merge \\
        --out ~/mmission_ws/routes/full_course.csv \\
        ~/mmission_ws/routes/01_start_slope.csv \\
        ~/mmission_ws/routes/02_scourse.csv \\
        ~/mmission_ws/routes/03_tpark.csv

  또는 순수 파이썬으로:
    python3 -m mission_manager.route_merge --out full.csv a.csv b.csv c.csv

  --loop 를 주면 마지막에 loop:true 로 저장(출차 후 원점 복귀 코스용).
  --max-gap M 으로 이음새 허용 간격(m)을 바꾼다(기본 20).
"""
from __future__ import annotations

import argparse
import csv
import math
import sys
from datetime import datetime, timezone
from pathlib import Path

import yaml

from .geo_utils import latlon_to_xy


COLUMNS = ("index", "latitude", "longitude", "x_m", "y_m",
           "direction", "mode", "drive_level")


def _read_one(csv_path: Path):
    """CSV 한 개를 읽어 (rows, origin_lat, origin_lon) 반환."""
    meta_path = csv_path.with_suffix(".yaml")
    if not csv_path.is_file():
        raise FileNotFoundError(f"CSV 없음: {csv_path}")
    if not meta_path.is_file():
        raise FileNotFoundError(f"메타데이터(.yaml) 없음: {meta_path}")
    with meta_path.open(encoding="utf-8") as stream:
        meta = yaml.safe_load(stream) or {}
    origin_lat = float(meta["origin_lat"])
    origin_lon = float(meta["origin_lon"])
    rows = []
    with csv_path.open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        for row in reader:
            rows.append(row)
    if not rows:
        raise ValueError(f"빈 경로: {csv_path}")
    return rows, origin_lat, origin_lon


def merge_routes(inputs, out_path, *, loop=False, max_gap_m=20.0):
    """
    여러 CSV를 순서대로 이어붙여 out_path에 저장.
    반환: (총 점 개수, 큰 이음새 간격 리스트[(index, gap_m)])
    """
    inputs = [Path(p) for p in inputs]
    out_path = Path(out_path)
    if len(inputs) < 1:
        raise ValueError("최소 1개 이상의 입력 CSV가 필요하다")

    # 첫 CSV의 origin을 전체 기준으로 삼는다.
    _first_rows, base_lat, base_lon = _read_one(inputs[0])

    merged = []      # 최종 행(dict) 목록
    prev_xy = None

    for csv_path in inputs:
        rows, _olat, _olon = _read_one(csv_path)
        for row in rows:
            lat = float(row["latitude"])
            lon = float(row["longitude"])
            # 모든 점을 '첫 CSV origin' 기준으로 재계산 → 좌표계 통일
            x, y = latlon_to_xy(lat, lon, base_lat, base_lon)
            if prev_xy is not None:
                gap = math.hypot(x - prev_xy[0], y - prev_xy[1])
                if gap < 1.0e-6:
                    # 완전 중복점은 건너뛴다(loader가 거부함).
                    continue
            merged.append({
                "latitude": f"{lat:.10f}", "longitude": f"{lon:.10f}",
                "x_m": f"{x:.3f}", "y_m": f"{y:.3f}",
                "direction": str(int(float(row["direction"]))),
                "mode": str(row["mode"]).strip(),
                "drive_level": f"{float(row['drive_level']):.2f}",
            })
            prev_xy = (x, y)

    # index 다시 매기기 + 이음새 간격 검사
    gaps_report = []
    out_rows = []
    for i, row in enumerate(merged):
        row_out = {"index": str(i)}
        row_out.update(row)
        out_rows.append(row_out)
        if i > 0:
            x0, y0 = float(out_rows[i - 1]["x_m"]), float(out_rows[i - 1]["y_m"])
            x1, y1 = float(row["x_m"]), float(row["y_m"])
            g = math.hypot(x1 - x0, y1 - y0)
            if g > max_gap_m:
                gaps_report.append((i, g))

    # 저장
    out_path.parent.mkdir(parents=True, exist_ok=True)
    with out_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=COLUMNS)
        writer.writeheader()
        writer.writerows(out_rows)

    meta = {
        "format_version": 1,
        "origin_lat": base_lat,
        "origin_lon": base_lon,
        "loop": bool(loop),
        "created_at": datetime.now(timezone.utc).isoformat(),
    }
    with out_path.with_suffix(".yaml").open("w", encoding="utf-8") as stream:
        yaml.safe_dump(meta, stream, sort_keys=False)

    return len(out_rows), gaps_report


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="GPS 모범경로 CSV 이어붙이기")
    parser.add_argument("inputs", nargs="+", help="이어붙일 CSV들(순서대로)")
    parser.add_argument("--out", required=True, help="출력 CSV 경로")
    parser.add_argument("--loop", action="store_true", help="loop 경로로 저장")
    parser.add_argument("--max-gap", type=float, default=20.0,
                        help="이음새 허용 간격(m), 기본 20")
    args = parser.parse_args(argv if argv is not None else sys.argv[1:])

    count, gaps = merge_routes(args.inputs, args.out,
                               loop=args.loop, max_gap_m=args.max_gap)
    print(f"[route_merge] {len(args.inputs)}개 파일 → {count}개 점 저장: {args.out}")
    if gaps:
        print("⚠️  이음새 간격이 너무 큰 지점(경로가 물리적으로 안 이어짐):")
        for idx, g in gaps:
            print(f"    index {idx}: {g:.2f} m (허용 {args.max_gap} m 초과)")
        print("    → 해당 구간 경로 끝/시작점이 실제로 이어지도록 다시 기록하세요.")
        return 1
    print("이음새 정상. route_loader 통과 가능.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
