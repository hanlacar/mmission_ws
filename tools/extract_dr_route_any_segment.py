#!/usr/bin/env python3

import argparse
import csv
import math
from pathlib import Path


def load_rows(path):
    with open(path, newline="", encoding="utf-8-sig") as f:
        reader = csv.DictReader(f)
        rows = list(reader)
        fields = set(reader.fieldnames or [])

    required = {
        "segment_id",
        "point_index",
        "x_m",
        "y_m",
        "direction",
        "mode",
        "drive_level",
    }

    missing = required - fields
    if missing:
        raise RuntimeError(
            "입력 CSV에 필요한 컬럼이 없습니다: "
            + ", ".join(sorted(missing))
        )

    for r in rows:
        r["_segment_id"] = str(r["segment_id"]).strip()
        r["_point_index"] = int(float(r["point_index"]))
        r["_x"] = float(r["x_m"])
        r["_y"] = float(r["y_m"])
        r["_direction"] = 1 if int(float(r.get("direction", 1) or 1)) >= 0 else -1
        r["_mode"] = int(float(r.get("mode", 1) or 1))
        r["_drive_level"] = float(r.get("drive_level", 2.0) or 2.0)
        r["_event"] = str(r.get("event", "NONE") or "NONE").strip()

    return rows


def build_segment_map(rows):
    segs = {}
    for r in rows:
        segs.setdefault(r["_segment_id"], []).append(r)

    for sid in segs:
        segs[sid].sort(key=lambda r: r["_point_index"])

    return segs


def list_segments(segs):
    print("=== 사용 가능한 segment ===")
    for sid, pts in segs.items():
        modes = sorted(set(p["_mode"] for p in pts))
        print(
            f"{sid:12s}  points={len(pts):4d}  "
            f"mode={','.join(map(str, modes))}"
        )


def filter_modes(points, start_mode, end_mode):
    result = []
    for p in points:
        if start_mode is not None and p["_mode"] < start_mode:
            continue
        if end_mode is not None and p["_mode"] > end_mode:
            continue
        result.append(p)
    return result


def point_distance(a, b):
    return math.hypot(
        a["_x"] - b["_x"],
        a["_y"] - b["_y"],
    )


def append_segment(
    combined,
    points,
    segment_id,
    max_join_gap,
    allow_gap,
):
    if not points:
        return

    if combined:
        gap = point_distance(
            combined[-1]["row"],
            points[0],
        )

        print(
            f"[JOIN] {combined[-1]['segment_id']} -> "
            f"{segment_id}: gap={gap:.3f} m"
        )

        if gap > max_join_gap and not allow_gap:
            raise RuntimeError(
                f"segment 연결 간격이 너무 큽니다: "
                f"{combined[-1]['segment_id']} -> {segment_id} "
                f"= {gap:.3f} m\n"
                f"--allow-gap 을 사용하거나 segment 순서를 확인하세요."
            )

        # 사실상 같은 접속점이면 두 번째 segment 첫 점 제거
        if gap < 1.0e-6:
            points = points[1:]

    for p in points:
        combined.append({
            "row": p,
            "segment_id": segment_id,
        })


def remove_consecutive_duplicates(combined):
    if len(combined) < 2:
        return combined

    out = [combined[0]]

    for item in combined[1:]:
        a = out[-1]["row"]
        b = item["row"]

        d = math.hypot(
            b["_x"] - a["_x"],
            b["_y"] - a["_y"],
        )

        if d > 1.0e-6:
            out.append(item)

    return out


def compute_yaws(combined):
    n = len(combined)

    if n < 2:
        return [0.0] * n

    yaws = []

    for i in range(n - 1):
        a = combined[i]["row"]
        b = combined[i + 1]["row"]

        yaw = math.degrees(
            math.atan2(
                b["_y"] - a["_y"],
                b["_x"] - a["_x"],
            )
        )

        yaws.append(yaw)

    yaws.append(yaws[-1])

    return yaws


def save_dr_csv(combined, output):
    yaws = compute_yaws(combined)

    fieldnames = [
        "index",
        "dr_x_m",
        "dr_y_m",
        "dr_yaw_deg",
        "direction",
        "mode",
        "drive_level",
        "event",
        "source_segment_id",
        "source_point_index",
    ]

    output.parent.mkdir(
        parents=True,
        exist_ok=True,
    )

    with open(
        output,
        "w",
        newline="",
        encoding="utf-8",
    ) as f:

        writer = csv.DictWriter(
            f,
            fieldnames=fieldnames,
        )

        writer.writeheader()

        for i, (item, yaw) in enumerate(
            zip(combined, yaws)
        ):
            p = item["row"]

            writer.writerow({
                "index": i,
                "dr_x_m": f"{p['_x']:.6f}",
                "dr_y_m": f"{p['_y']:.6f}",
                "dr_yaw_deg": f"{yaw:.6f}",
                "direction": p["_direction"],
                "mode": p["_mode"],
                "drive_level": p["_drive_level"],
                "event": p["_event"],
                "source_segment_id": item["segment_id"],
                "source_point_index": p["_point_index"],
            })


def main():
    parser = argparse.ArgumentParser(
        description=(
            "segment 기반 route_network CSV에서 "
            "원하는 segment 또는 여러 segment를 골라 "
            "DR follower용 CSV로 추출합니다."
        )
    )

    parser.add_argument(
        "input_csv",
        help="route_network_segmented_*.csv",
    )

    parser.add_argument(
        "--segment",
        nargs="+",
        help=(
            "추출할 segment를 주행 순서대로 입력. "
            "예: --segment START_A COMMON_1 T_A"
        ),
    )

    parser.add_argument(
        "--start-mode",
        type=int,
        default=None,
        help="이 mode 이상만 추출",
    )

    parser.add_argument(
        "--end-mode",
        type=int,
        default=None,
        help="이 mode 이하만 추출",
    )

    parser.add_argument(
        "--output",
        help="출력 DR CSV 경로",
    )

    parser.add_argument(
        "--list",
        action="store_true",
        help="사용 가능한 segment와 mode만 출력",
    )

    parser.add_argument(
        "--max-join-gap",
        type=float,
        default=1.5,
        help=(
            "여러 segment 연결 시 허용할 최대 접속 간격(m). "
            "기본 1.5m"
        ),
    )

    parser.add_argument(
        "--allow-gap",
        action="store_true",
        help="큰 segment 연결 간격이 있어도 강제로 저장",
    )

    args = parser.parse_args()

    path = Path(
        args.input_csv
    ).expanduser()

    if not path.is_file():
        raise SystemExit(
            f"입력 파일 없음: {path}"
        )

    rows = load_rows(path)
    segs = build_segment_map(rows)

    if args.list:
        list_segments(segs)
        return

    if not args.segment:
        raise SystemExit(
            "--segment를 하나 이상 지정하세요. "
            "먼저 --list로 확인할 수 있습니다."
        )

    if not args.output:
        raise SystemExit(
            "--output 경로를 지정하세요."
        )

    if (
        args.start_mode is not None
        and not (1 <= args.start_mode <= 11)
    ):
        raise SystemExit(
            "--start-mode는 1~11이어야 합니다."
        )

    if (
        args.end_mode is not None
        and not (1 <= args.end_mode <= 11)
    ):
        raise SystemExit(
            "--end-mode는 1~11이어야 합니다."
        )

    if (
        args.start_mode is not None
        and args.end_mode is not None
        and args.start_mode > args.end_mode
    ):
        raise SystemExit(
            "start-mode가 end-mode보다 클 수 없습니다."
        )

    combined = []

    for sid in args.segment:
        if sid not in segs:
            print()
            list_segments(segs)
            raise SystemExit(
                f"\n존재하지 않는 segment: {sid}"
            )

        pts = filter_modes(
            list(segs[sid]),
            args.start_mode,
            args.end_mode,
        )

        if not pts:
            print(
                f"[SKIP] {sid}: 지정한 mode 범위에 점이 없습니다."
            )
            continue

        print(
            f"[ADD] {sid}: {len(pts)} points "
            f"(mode {pts[0]['_mode']}~{pts[-1]['_mode']})"
        )

        append_segment(
            combined,
            pts,
            sid,
            args.max_join_gap,
            args.allow_gap,
        )

    combined = remove_consecutive_duplicates(
        combined
    )

    if len(combined) < 2:
        raise SystemExit(
            "최종 경로가 2점 미만입니다."
        )

    out = Path(
        args.output
    ).expanduser()

    save_dr_csv(
        combined,
        out,
    )

    # 최종 최대 점 간격 검증
    max_step = 0.0
    max_pair = None

    for i in range(1, len(combined)):
        a = combined[i - 1]["row"]
        b = combined[i]["row"]

        d = math.hypot(
            b["_x"] - a["_x"],
            b["_y"] - a["_y"],
        )

        if d > max_step:
            max_step = d
            max_pair = (
                combined[i - 1]["segment_id"],
                combined[i]["segment_id"],
            )

    print()
    print("=== DR 경로 추출 완료 ===")
    print(
        "segments : "
        + " -> ".join(args.segment)
    )
    print(
        f"points   : {len(combined)}"
    )
    print(
        f"max step : {max_step:.3f} m"
    )

    if max_pair:
        print(
            f"max pair : {max_pair[0]} -> {max_pair[1]}"
        )

    print(
        f"output   : {out}"
    )


if __name__ == "__main__":
    main()
