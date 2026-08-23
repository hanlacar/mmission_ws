#!/usr/bin/env python3
"""
analyze_entry_dirs — 저장된 8자 경로 CSV에서 교차로 진입 방위(N/E/S/W)를 오프라인 분석.

목적:
  8자형 경로가 교차로(원: center+radius)를 4번(N/E/S/W) 서로 다른 방위로 통과하는지
  사람이 검증하기 위한 도구. 각 "원 밖 → 원 안" 전이(진입 이벤트)에서 진행방향을
  구해 나침반 방위로 양자화하고, 방위별 진입 횟수 요약과 intersection.py용
  plan_N/E/S/W 템플릿 초안을 출력한다.

  좌표/방위 변환은 geo_utils, intersection 모듈의 기존 함수를 그대로 재사용한다
  (새 변환 로직 없음). route_loader.load_route()로 CSV+YAML을 검증 로드한다.

★ 순수 오프라인 스크립트. rclpy/ROS 런타임에 의존하지 않는다 ★ (GPS 하드웨어 불필요)

실행 예 (routes/test/ 안의 기존 샘플 CSV로 스모크 테스트, colcon build 후):
  ros2 run mission_manager analyze_entry_dirs \
      --route src/mission_manager/routes/test/06_self_intersection.csv --center 0 0 --radius 1.0

  colcon build 없이 소스만으로 돌려볼 때는 패키지 상대import 때문에 -m 모듈 실행이 필요:
  python3 -m mission_manager.analyze_entry_dirs \
      --route routes/test/06_self_intersection.csv --center 0 0 --radius 1.0
  (mission_manager/ 상위 디렉터리, 즉 mission_manager 패키지가 보이는 위치에서 실행)
"""
import argparse
import math
import sys

from .geo_utils import dist_m
from .intersection import COMPASS_DIRS, compass_deg_to_dir, math_heading_to_compass_deg
from .route_loader import RouteValidationError, load_route
from .route_model import Direction


def _estimate_entry_heading(waypoints, entry_idx):
    """진입 waypoint 인덱스 부근(앞뒤 인접점)의 위치차로 heading(rad, 수학관례) 추정.
    direction == REVERSE 구간이면 차체 진행방향으로 보정하기 위해 heading에 π를 더한다."""
    lo = max(entry_idx - 1, 0)
    hi = min(entry_idx + 1, len(waypoints) - 1)
    if lo == hi:
        # 경로가 너무 짧아 앞뒤 점을 못 구하는 예외적인 경우: 가능한 이웃 하나만 사용
        lo = max(entry_idx - 1, 0)
        hi = entry_idx if entry_idx != lo else min(entry_idx + 1, len(waypoints) - 1)
    p0, p1 = waypoints[lo], waypoints[hi]
    dx = p1.x_m - p0.x_m
    dy = p1.y_m - p0.y_m
    heading = math.atan2(dy, dx)
    if waypoints[entry_idx].direction == Direction.REVERSE:
        heading += math.pi
    return heading


def find_entry_events(route, cx, cy, radius):
    """웨이포인트를 순서대로 훑어 '원 밖 → 원 안' 전이를 진입 이벤트로 반환.
    각 이벤트: dict(index, x, y, compass_deg, dir)."""
    waypoints = route.waypoints
    inside = [dist_m(wp.x_m, wp.y_m, cx, cy) <= radius for wp in waypoints]

    events = []
    for i in range(1, len(waypoints)):
        if not inside[i - 1] and inside[i]:
            heading = _estimate_entry_heading(waypoints, i)
            compass = math_heading_to_compass_deg(heading)
            d = compass_deg_to_dir(compass)
            wp = waypoints[i]
            events.append({
                "index": wp.index,
                "x": wp.x_m,
                "y": wp.y_m,
                "compass_deg": compass,
                "dir": d,
            })
    return events


def _print_report(events):
    print(f"{'#':>3}  {'wp_idx':>6}  {'x':>8}  {'y':>8}  {'compass_deg':>11}  dir")
    print("-" * 52)
    for n, ev in enumerate(events, start=1):
        print(f"{n:>3}  {ev['index']:>6}  {ev['x']:>8.3f}  {ev['y']:>8.3f}  "
              f"{ev['compass_deg']:>11.1f}  {ev['dir']}")

    counts = {d: 0 for d in COMPASS_DIRS}
    for ev in events:
        counts[ev["dir"]] += 1

    print()
    summary = "  ".join(f"{d}:{counts[d]}" for d in COMPASS_DIRS)
    print(f"방위별 진입 횟수 요약: {summary}")

    overlapped = [d for d in COMPASS_DIRS if counts[d] >= 2]
    if overlapped:
        print()
        print(f"⚠ 경고: 방위 {', '.join(overlapped)} 는 2회 이상 진입 감지됨. "
              "해당 방위는 plan 배열로 회차를 구분해야 함 "
              "(예: plan_N: [\"straight\", \"right\"] → 1회차/2회차).")

    print()
    print("plan 템플릿 초안 (복붙 후 회차별 기동을 채우세요: straight/left/right):")
    print("ix_center:")
    for d in COMPASS_DIRS:
        n = counts[d]
        if n == 0:
            print(f"  # plan_{d}: []   # 이 방위 진입 미감지")
        else:
            placeholders = ", ".join('"?"' for _ in range(n))
            print(f"  plan_{d}: [{placeholders}]   # {n}회차 기동을 채우세요")


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="8자 경로 CSV에서 교차로 진입 방위(N/E/S/W)를 오프라인 분석")
    parser.add_argument("--route", required=True, help="경로 CSV 경로 (.yaml 메타데이터가 옆에 있어야 함)")
    parser.add_argument("--center", required=True, nargs=2, type=float, metavar=("CX", "CY"),
                         help="교차로 중심 로컬 좌표(m): cx cy")
    parser.add_argument("--radius", type=float, default=2.5, help="진입 판정 반경(m, 기본 2.5)")
    args = parser.parse_args(argv)

    try:
        route = load_route(args.route)
    except RouteValidationError as exc:
        print(f"route 로드 실패: {exc}", file=sys.stderr)
        return 1

    cx, cy = args.center
    events = find_entry_events(route, cx, cy, args.radius)

    if not events:
        print("진입 이벤트를 하나도 찾지 못했습니다. --center/--radius 값을 확인하세요.")
        return 0

    _print_report(events)
    return 0


if __name__ == "__main__":
    sys.exit(main())
