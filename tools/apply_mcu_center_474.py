#!/usr/bin/env python3
"""Safely migrate T870 steering calibration from center ADC 496 to 474.

Contract kept unchanged:
- 18 ADC counts/degree
- LEFT = positive
- RIGHT = negative
- steering clamp = ±22 degrees

This script changes only named steering-center calibration fields and generates
an authoritative center-474 steering table. It intentionally does NOT modify
encoder, drive, IMU, DDS, ROS domain, electrical ADC limits, or polarity.

Usage:
    python3 tools/apply_mcu_center_474.py ~/mcu_ws_takeover
"""

from __future__ import annotations

import csv
import re
import shutil
import sys
from pathlib import Path

OLD_CENTER = 496
NEW_CENTER = 474
COUNTS_PER_DEG = 18.0
MAX_DEG = 22

TEXT_SUFFIXES = {
    ".py", ".yaml", ".yml", ".ino", ".cpp", ".cc", ".c", ".hpp", ".h",
    ".launch", ".md", ".txt", ".sh", ".cfg",
}

CALIBRATION_PATTERNS = [
    (re.compile(r"(\bsteer_center_adc\s*:\s*)496\b"),
     r"\g<1>474", "steer_center_adc YAML"),
    (re.compile(r"(\bsteering_center_adc\s*:\s*)496\b"),
     r"\g<1>474", "steering_center_adc YAML"),
    (re.compile(r"(\bcenter_adc\s*:\s*)496\b"),
     r"\g<1>474", "center_adc YAML"),
    (re.compile(r"(declare_parameter\(\s*[\"']steer_center_adc[\"']\s*,\s*)496(\s*\))"),
     r"\g<1>474\g<2>", "steer_center_adc ROS default"),
    (re.compile(r"(\bSTEER_CENTER_ADC\s*=\s*)496\b"),
     r"\g<1>474", "STEER_CENTER_ADC"),
    (re.compile(r"(\bSTEER_CENTER_DEFAULT\s*=\s*)496\b"),
     r"\g<1>474", "STEER_CENTER_DEFAULT"),
    (re.compile(r"(\bCENTER_ADC\s*=\s*)496\b"),
     r"\g<1>474", "CENTER_ADC"),
    (re.compile(r"(\bCENTER\s*=\s*)496\b"),
     r"\g<1>474", "CENTER"),
    (re.compile(r"(\bcenter_adc\s*=\s*)496\b"),
     r"\g<1>474", "center_adc"),
    (re.compile(r"(\bcenterAdc\s*=\s*)496\b"),
     r"\g<1>474", "centerAdc"),
    (re.compile(r"(CFG,CENTER,)496\b"),
     r"\g<1>474", "CFG CENTER literal"),
]

VERIFY_PATTERNS = [
    re.compile(r"\bsteer_center_adc\s*:\s*496\b"),
    re.compile(r"\bsteering_center_adc\s*:\s*496\b"),
    re.compile(r"\bcenter_adc\s*:\s*496\b"),
    re.compile(r"declare_parameter\(\s*[\"']steer_center_adc[\"']\s*,\s*496\s*\)"),
    re.compile(r"\bSTEER_CENTER_ADC\s*=\s*496\b"),
    re.compile(r"\bSTEER_CENTER_DEFAULT\s*=\s*496\b"),
    re.compile(r"\bCENTER_ADC\s*=\s*496\b"),
    re.compile(r"\bcenter_adc\s*=\s*496\b"),
    re.compile(r"\bcenterAdc\s*=\s*496\b"),
]

def adc_for_deg(deg: float) -> int:
    return int(round(NEW_CENTER + COUNTS_PER_DEG * float(deg)))

def validate_contract() -> None:
    if adc_for_deg(0) != 474:
        raise RuntimeError("center validation failed")
    if adc_for_deg(-22) != 78 or adc_for_deg(22) != 870:
        raise RuntimeError("±22 degree endpoint validation failed")
    if not (0 <= adc_for_deg(-22) <= 1023 and 0 <= adc_for_deg(22) <= 1023):
        raise RuntimeError("ADC target exceeds 10-bit ADC range")

def backup_file(path: Path) -> None:
    backup = path.with_name(path.name + ".center496.bak")
    if not backup.exists():
        shutil.copy2(path, backup)

def patch_text_file(path: Path) -> list[str]:
    try:
        original = path.read_text(encoding="utf-8")
    except UnicodeDecodeError:
        return []

    text = original
    changes: list[str] = []
    for pattern, replacement, description in CALIBRATION_PATTERNS:
        text, count = pattern.subn(replacement, text)
        if count:
            changes.append(f"{description} x{count}")

    # Keep documentation/table references consistent without changing arbitrary
    # numeric values.
    if "steering_table_center496.csv" in text:
        text = text.replace(
            "steering_table_center496.csv",
            "steering_table_center474.csv",
        )
        changes.append("table filename reference")

    if text != original:
        backup_file(path)
        path.write_text(text, encoding="utf-8")
    return changes

def generate_table(root: Path) -> Path:
    candidates = list(root.rglob("steering_table_center496.csv"))
    if candidates:
        base_dir = candidates[0].parent
        old = candidates[0]
        backup_file(old)
    else:
        base_dir = root

    out = base_dir / "steering_table_center474.csv"
    with out.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(["steering_deg", "direction", "adc"])
        for deg in range(-MAX_DEG, MAX_DEG + 1):
            direction = "CENTER" if deg == 0 else ("LEFT" if deg > 0 else "RIGHT")
            writer.writerow([deg, direction, adc_for_deg(deg)])
    return out

def verify_workspace(root: Path) -> None:
    remaining = []
    saw_new_center = False
    saw_counts = False

    for path in root.rglob("*"):
        if not path.is_file() or path.suffix.lower() not in TEXT_SUFFIXES:
            continue
        try:
            text = path.read_text(encoding="utf-8")
        except UnicodeDecodeError:
            continue

        low = text.lower()
        if "steer" in low or "steering" in low or "center_adc" in low:
            if any(p.search(text) for p in VERIFY_PATTERNS):
                remaining.append(str(path))
            if re.search(r"\b474\b", text):
                saw_new_center = True
            if re.search(r"18(?:\.0+)?", text) and ("count" in low or "adc" in low):
                saw_counts = True

    if remaining:
        raise RuntimeError(
            "Old center 496 still active in calibration fields:\n  "
            + "\n  ".join(remaining)
        )
    if not saw_new_center:
        raise RuntimeError("Could not verify center 474 in steering calibration files")
    if not saw_counts:
        raise RuntimeError("Could not verify 18 ADC counts/degree contract")

def main() -> int:
    validate_contract()

    if len(sys.argv) != 2:
        print("Usage: apply_mcu_center_474.py <mcu_workspace>")
        return 2

    root = Path(sys.argv[1]).expanduser().resolve()
    if not root.is_dir():
        print(f"[FAIL] workspace does not exist: {root}")
        return 2

    changed = []
    for path in root.rglob("*"):
        if not path.is_file() or path.suffix.lower() not in TEXT_SUFFIXES:
            continue
        notes = patch_text_file(path)
        if notes:
            changed.append((path, notes))

    if not changed:
        print("[FAIL] No active center-496 calibration field was found.")
        print("       Nothing was changed.")
        return 1

    table_path = generate_table(root)
    verify_workspace(root)

    print("[PASS] T870 steering center migration verified")
    print("  center ADC       : 496 -> 474")
    print("  counts/degree    : 18.0 (unchanged)")
    print("  direction        : LEFT=+, RIGHT=- (unchanged)")
    print("  steering limit   : ±22 deg (unchanged)")
    print("  target ADC range : 78 .. 870 (within 0..1023)")
    print(f"  table            : {table_path}")
    print()
    print("Changed calibration files:")
    for path, notes in changed:
        print(f"  {path}: {', '.join(notes)}")
    print()
    print("Key targets:")
    for deg in (-22, -20, -15, -10, -5, 0, 5, 10, 15, 20, 22):
        direction = "C" if deg == 0 else ("L" if deg > 0 else "R")
        print(f"  {direction:>1} {deg:+3d} deg -> ADC {adc_for_deg(deg)}")
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
