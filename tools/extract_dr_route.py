#!/usr/bin/env python3

import argparse
import math
from pathlib import Path

import numpy as np
import pandas as pd


def compute_yaw_deg(x, y):
    """
    각 waypoint에서 다음 waypoint 방향으로 yaw 계산.
    마지막 점은 직전 yaw 사용.
    """
    n = len(x)

    if n < 2:
        return np.zeros(n, dtype=float)

    yaw = np.zeros(n, dtype=float)

    dx = np.diff(x)
    dy = np.diff(y)

    seg_yaw = np.degrees(
        np.arctan2(dy, dx)
    )

    yaw[:-1] = seg_yaw
    yaw[-1] = seg_yaw[-1]

    return yaw


def remove_consecutive_duplicates(df, epsilon=1e-6):
    """
    연속된 동일 좌표 제거.
    """
    if len(df) <= 1:
        return df.copy()

    xy = df[
        ["x_m", "y_m"]
    ].to_numpy(dtype=float)

    dist = np.linalg.norm(
        np.diff(xy, axis=0),
        axis=1,
    )

    keep = np.ones(
        len(df),
        dtype=bool,
    )

    keep[1:] = (
        dist > epsilon
    )

    return (
        df.iloc[np.where(keep)[0]]
        .reset_index(drop=True)
    )


def main():
    parser = argparse.ArgumentParser(
        description=(
            "segment 기반 경로 CSV를 "
            "DR follower용 CSV로 변환"
        )
    )

    parser.add_argument(
        "input_csv",
        help="입력 route_network_segmented CSV",
    )

    parser.add_argument(
        "--output",
        required=True,
        help="출력 DR CSV",
    )

    parser.add_argument(
        "--segment",
        default="AAA_BASE",
        help=(
            "추출할 segment_id. "
            "기본값 AAA_BASE"
        ),
    )

    parser.add_argument(
        "--start-mode",
        type=int,
        default=None,
        help="시작 mode",
    )

    parser.add_argument(
        "--end-mode",
        type=int,
        default=None,
        help="끝 mode",
    )

    args = parser.parse_args()

    input_path = Path(
        args.input_csv
    ).expanduser()

    output_path = Path(
        args.output
    ).expanduser()

    if not input_path.is_file():
        raise SystemExit(
            f"입력 CSV 없음: {input_path}"
        )

    df = pd.read_csv(
        input_path
    )

    required = {
        "segment_id",
        "x_m",
        "y_m",
        "direction",
        "mode",
        "drive_level",
    }

    missing = (
        required
        - set(df.columns)
    )

    if missing:
        raise SystemExit(
            f"필수 컬럼 없음: {sorted(missing)}"
        )

    # --------------------------------------------------
    # segment 선택
    # --------------------------------------------------

    route = df[
        df["segment_id"].astype(str)
        == str(args.segment)
    ].copy()

    if route.empty:
        available = sorted(
            df["segment_id"]
            .astype(str)
            .unique()
        )

        raise SystemExit(
            f"segment 없음: {args.segment}\n"
            f"사용 가능: {available}"
        )

    # 원래 point 순서 유지
    if "point_index" in route.columns:
        route = route.sort_values(
            "point_index"
        )

    # --------------------------------------------------
    # mode 범위 선택
    # --------------------------------------------------

    if args.start_mode is not None:
        route = route[
            route["mode"]
            >= args.start_mode
        ]

    if args.end_mode is not None:
        route = route[
            route["mode"]
            <= args.end_mode
        ]

    route = route.reset_index(
        drop=True
    )

    if len(route) < 2:
        raise SystemExit(
            "추출된 경로가 2점 미만입니다."
        )

    # --------------------------------------------------
    # 중복 좌표 제거
    # --------------------------------------------------

    route = remove_consecutive_duplicates(
        route
    )

    if len(route) < 2:
        raise SystemExit(
            "중복 제거 후 경로가 2점 미만입니다."
        )

    # --------------------------------------------------
    # DR 좌표
    # --------------------------------------------------

    x = route[
        "x_m"
    ].astype(float).to_numpy()

    y = route[
        "y_m"
    ].astype(float).to_numpy()

    yaw_deg = compute_yaw_deg(
        x,
        y,
    )

    # --------------------------------------------------
    # DR follower용 CSV 생성
    # --------------------------------------------------

    out = pd.DataFrame({
        "index":
            np.arange(
                len(route),
                dtype=int,
            ),

        "dr_x_m":
            x,

        "dr_y_m":
            y,

        "dr_yaw_deg":
            yaw_deg,

        "direction":
            route["direction"]
            .fillna(1)
            .astype(int)
            .to_numpy(),

        "mode":
            route["mode"]
            .fillna(1)
            .astype(int)
            .to_numpy(),

        "drive_level":
            route["drive_level"]
            .fillna(2.0)
            .astype(float)
            .to_numpy(),

        "event":
            (
                route["event"]
                .fillna("NONE")
                .astype(str)
                .to_numpy()
                if "event"
                in route.columns
                else ["NONE"] * len(route)
            ),

        "source_segment_id":
            route[
                "segment_id"
            ].astype(str).to_numpy(),

        "source_point_index":
            (
                route["point_index"]
                .astype(int)
                .to_numpy()
                if "point_index"
                in route.columns
                else np.arange(
                    len(route),
                    dtype=int,
                )
            ),
    })

    # --------------------------------------------------
    # 연속 점 간격 검증
    # --------------------------------------------------

    pts = out[
        ["dr_x_m", "dr_y_m"]
    ].to_numpy()

    distances = np.linalg.norm(
        np.diff(pts, axis=0),
        axis=1,
    )

    max_step = float(
        distances.max()
    )

    mean_step = float(
        distances.mean()
    )

    # --------------------------------------------------
    # 저장
    # --------------------------------------------------

    output_path.parent.mkdir(
        parents=True,
        exist_ok=True,
    )

    out.to_csv(
        output_path,
        index=False,
        float_format="%.6f",
    )

    print()
    print("=== DR 경로 추출 완료 ===")
    print(
        f"입력        : {input_path}"
    )
    print(
        f"segment     : {args.segment}"
    )

    if (
        args.start_mode is not None
        or args.end_mode is not None
    ):
        print(
            "mode        : "
            f"{args.start_mode}"
            f" ~ "
            f"{args.end_mode}"
        )

    print(
        f"waypoints   : {len(out)}"
    )

    print(
        f"평균 점간격 : "
        f"{mean_step:.3f} m"
    )

    print(
        f"최대 점간격 : "
        f"{max_step:.3f} m"
    )

    print(
        f"출력        : {output_path}"
    )

    if max_step > 2.0:
        print(
            "WARNING: 2m 이상 점프가 있습니다."
        )


if __name__ == "__main__":
    main()
