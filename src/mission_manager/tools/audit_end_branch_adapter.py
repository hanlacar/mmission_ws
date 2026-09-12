#!/usr/bin/env python3
from pathlib import Path
import ast
import sys

root = Path(__file__).resolve().parents[1]
node = root / "mission_manager" / "end_branch_adapter_node.py"
cfg = root / "config" / "end_branch_adapter.yaml"

src = node.read_text()
ast.parse(src)
cfg_src = cfg.read_text()

checks = {
    "mode 11 gate": 'active_mode", "11"' in src,
    "fused aspect input":
        '"/camera/traffic_light_fused/aspect"' in src,
    "confidence input":
        '"/camera/traffic_light_fused/confidence"' in src,
    "diagnostics input":
        '"/camera/traffic_light_fused/diagnostics"' in src,
    "END output": '"/dr/end_branch"' in src,
    "GREEN_DOWN supported": '"GREEN_DOWN"' in src,
    "GREEN_LEFT supported": '"GREEN_LEFT"' in src,
    "RGB DOWN verification":
        "rgb_green_down_verified" in src,
    "confidence threshold":
        "minimum_confidence" in src,
    "distinct-frame confirmation":
        "last_counted_stamp" in src and "confirm_frames" in src,
    "fail closed on stale":
        "CAMERA_INPUT_STALE_FAIL_CLOSED" in src,
    "mapping unconfigured by default":
        "green_down_branch: NONE" in cfg_src
        and "green_left_branch: NONE" in cfg_src,
    "no drive output":
        "/gps_drive" not in src and "/lidar_drive" not in src,
    "no wheel output":
        "/gps_wheel" not in src and "/lidar_wheel" not in src,
}

passed = 0
for name, ok in checks.items():
    print(f"{'PASS' if ok else 'FAIL'} | {name}")
    passed += int(ok)

print(f"\nFINAL: {passed}/{len(checks)} PASS")
sys.exit(0 if passed == len(checks) else 1)
