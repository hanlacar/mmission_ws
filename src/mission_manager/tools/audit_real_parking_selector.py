#!/usr/bin/env python3
from pathlib import Path
import ast
import sys

root = Path(__file__).resolve().parents[1]
node = root / "mission_manager" / "real_parking_slot_selector_node.py"
cfg = root / "config" / "real_parking_slot_selector.yaml"

src = node.read_text()
ast.parse(src)

checks = {
    "front LaserScan": '"/scan_front"' in src,
    "rear LaserScan": '"/scan_rear"' in src,
    "T output": '"/t_parking/selected_slot"' in src,
    "V output": '"/parallel_parking/selected_slot"' in src,
    "mode input": '"/drive_mode"' in src,
    "T_A/T_B contract": '"T_A"' in src and '"T_B"' in src,
    "V_A/V_B contract": '"V_A"' in src and '"V_B"' in src,
    "fail-closed ROI": "ROI_NOT_CONFIGURED_FAIL_CLOSED" in src,
    "scan stale fail-closed": "SCAN_STALE_FAIL_CLOSED" in src,
    "no map subscription": "OccupancyGrid" not in src and '"/map"' not in src,
    "no cmd_vel": "Twist" not in src and '"/cmd_vel"' not in src,
    "no drive output": '"/lidar_drive"' not in src and '"/gps_drive"' not in src,
    "no wheel output": '"/lidar_wheel"' not in src and '"/gps_wheel"' not in src,
    "no obstacle_layout": "obstacle_layout" not in src,
    "config exists": cfg.is_file(),
    "all ROI defaults disabled":
        cfg.read_text().count("_enabled: false") == 8,
}

passed = 0
for name, ok in checks.items():
    print(f"{'PASS' if ok else 'FAIL'} | {name}")
    passed += int(ok)

print(f"\nFINAL: {passed}/{len(checks)} PASS")
sys.exit(0 if passed == len(checks) else 1)
