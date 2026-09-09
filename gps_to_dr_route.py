#!/usr/bin/env python3

import csv
import math
import sys
from pathlib import Path


RESAMPLE_STEP_M = 0.20
YAW_LOOKAHEAD_M = 0.80
SMOOTH_WINDOW = 5


def moving_average(values, window):
    if window <= 1:
        return values[:]

    out = []

    half = window // 2

    for i in range(len(values)):
        start = max(0, i - half)
        end = min(len(values), i + half + 1)

        chunk = values[start:end]

        out.append(
            sum(chunk) / len(chunk)
        )

    return out


def cumulative_distances(xs, ys):
    dists = [0.0]

    for i in range(1, len(xs)):
        dx = xs[i] - xs[i - 1]
        dy = ys[i] - ys[i - 1]

        dists.append(
            dists[-1] + math.hypot(dx, dy)
        )

    return dists


def interpolate_at_distance(xs, ys, dists, target):
    if target <= 0.0:
        return xs[0], ys[0]

    if target >= dists[-1]:
        return xs[-1], ys[-1]

    for i in range(1, len(dists)):
        if dists[i] >= target:
            d0 = dists[i - 1]
            d1 = dists[i]

            if abs(d1 - d0) < 1e-9:
                return xs[i], ys[i]

            ratio = (target - d0) / (d1 - d0)

            x = xs[i - 1] + ratio * (xs[i] - xs[i - 1])
            y = ys[i - 1] + ratio * (ys[i] - ys[i - 1])

            return x, y

    return xs[-1], ys[-1]


def nearest_original_row(rows, xs, ys, x, y):
    best_i = 0
    best_d = float("inf")

    for i, (px, py) in enumerate(zip(xs, ys)):
        d = (px - x) ** 2 + (py - y) ** 2

        if d < best_d:
            best_d = d
            best_i = i

    return rows[best_i]


def main():
    if len(sys.argv) != 3:
        print(
            "사용법: python3 gps_to_dr_route.py "
            "입력.csv 출력.csv"
        )
        sys.exit(1)

    input_csv = Path(sys.argv[1]).expanduser()
    output_csv = Path(sys.argv[2]).expanduser()

    if not input_csv.exists():
        print(f"입력 파일 없음: {input_csv}")
        sys.exit(1)

    with input_csv.open(
        "r",
        encoding="utf-8",
        newline="",
    ) as f:
        rows = list(csv.DictReader(f))

    if len(rows) < 2:
        print("경로점이 너무 적습니다.")
        sys.exit(1)

    required = {
        "x_m",
        "y_m",
    }

    missing = required - set(rows[0].keys())

    if missing:
        print(
            "필수 컬럼 없음:",
            ", ".join(sorted(missing)),
        )
        sys.exit(1)

    raw_x = [
        float(row["x_m"])
        for row in rows
    ]

    raw_y = [
        float(row["y_m"])
        for row in rows
    ]

    # ----------------------------------------------------------
    # 1. GPS 위치 노이즈 완화
    # ----------------------------------------------------------

    smooth_x = moving_average(
        raw_x,
        SMOOTH_WINDOW,
    )

    smooth_y = moving_average(
        raw_y,
        SMOOTH_WINDOW,
    )

    # ----------------------------------------------------------
    # 2. 누적거리 계산
    # ----------------------------------------------------------

    dists = cumulative_distances(
        smooth_x,
        smooth_y,
    )

    total_length = dists[-1]

    if total_length < RESAMPLE_STEP_M:
        print("경로 길이가 너무 짧습니다.")
        sys.exit(1)

    # ----------------------------------------------------------
    # 3. 일정 거리 간격으로 재샘플링
    # ----------------------------------------------------------

    sample_distances = []

    d = 0.0

    while d < total_length:
        sample_distances.append(d)
        d += RESAMPLE_STEP_M

    if sample_distances[-1] < total_length:
        sample_distances.append(total_length)

    sample_x = []
    sample_y = []

    for target in sample_distances:
        x, y = interpolate_at_distance(
            smooth_x,
            smooth_y,
            dists,
            target,
        )

        sample_x.append(x)
        sample_y.append(y)

    # ----------------------------------------------------------
    # 4. 시작 방향 계산
    #    첫 점 바로 다음 점이 아니라 0.8m 앞을 사용
    # ----------------------------------------------------------

    start_target = min(
        YAW_LOOKAHEAD_M,
        total_length,
    )

    start_x2, start_y2 = interpolate_at_distance(
        sample_x,
        sample_y,
        cumulative_distances(
            sample_x,
            sample_y,
        ),
        start_target,
    )

    heading0 = math.atan2(
        start_y2 - sample_y[0],
        start_x2 - sample_x[0],
    )

    c = math.cos(heading0)
    s = math.sin(heading0)

    x0 = sample_x[0]
    y0 = sample_y[0]

    converted = []

    sample_dists = cumulative_distances(
        sample_x,
        sample_y,
    )

    # ----------------------------------------------------------
    # 5. DR 상대좌표 + 안정적인 yaw 계산
    # ----------------------------------------------------------

    for i in range(len(sample_x)):

        gx = sample_x[i]
        gy = sample_y[i]

        dx = gx - x0
        dy = gy - y0

        dr_x = (
            c * dx
            + s * dy
        )

        dr_y = (
            -s * dx
            + c * dy
        )

        # 현재점에서 약 0.8m 앞 지점을 보고 yaw 계산
        target_dist = min(
            sample_dists[i] + YAW_LOOKAHEAD_M,
            sample_dists[-1],
        )

        tx, ty = interpolate_at_distance(
            sample_x,
            sample_y,
            sample_dists,
            target_dist,
        )

        if (
            abs(tx - gx) < 1e-9
            and abs(ty - gy) < 1e-9
        ):
            if converted:
                yaw_deg = float(
                    converted[-1]["yaw_deg"]
                )
            else:
                yaw_deg = 0.0

        else:
            yaw = math.atan2(
                ty - gy,
                tx - gx,
            ) - heading0

            yaw = math.atan2(
                math.sin(yaw),
                math.cos(yaw),
            )

            yaw_deg = math.degrees(yaw)

        original = nearest_original_row(
            rows,
            smooth_x,
            smooth_y,
            gx,
            gy,
        )

        direction = original.get(
            "direction",
            "1",
        )

        mode = original.get(
            "mode",
            "1",
        )

        drive_level = original.get(
            "drive_level",
            "2.0",
        )

        event = original.get(
            "event",
            "NONE",
        )

        converted.append(
            {
                "index": i,
                "x_m": f"{dr_x:.3f}",
                "y_m": f"{dr_y:.3f}",
                "yaw_deg": f"{yaw_deg:.3f}",
                "direction": direction,
                "mode": mode,
                "drive_level": drive_level,
                "event": event,
            }
        )

    output_csv.parent.mkdir(
        parents=True,
        exist_ok=True,
    )

    fieldnames = [
        "index",
        "x_m",
        "y_m",
        "yaw_deg",
        "direction",
        "mode",
        "drive_level",
        "event",
    ]

    with output_csv.open(
        "w",
        encoding="utf-8",
        newline="",
    ) as f:

        writer = csv.DictWriter(
            f,
            fieldnames=fieldnames,
        )

        writer.writeheader()
        writer.writerows(converted)

    print(f"변환 완료: {output_csv}")
    print(f"원본 경로점: {len(rows)}")
    print(f"변환 경로점: {len(converted)}")
    print(f"경로 길이: {total_length:.2f} m")
    print(
        f"시작 방향: "
        f"{math.degrees(heading0):.2f} deg"
    )
    print(
        f"재샘플링 간격: "
        f"{RESAMPLE_STEP_M:.2f} m"
    )
    print(
        f"Yaw lookahead: "
        f"{YAW_LOOKAHEAD_M:.2f} m"
    )


if __name__ == "__main__":
    main()