#!/usr/bin/env python3
import argparse
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

# --- All helper functions remain the same ---
HOME = Path.home()
RACEMAN = HOME / ".torcs" / "config" / "raceman"
OUT_XML = RACEMAN / "overtake_step1.xml"
DEFAULT_TRACK = ("g-track-1", "road")
VALID_D = {80, 100, 120, 140, 160, 180, 200, 220, 240, 260, 280, 300}

def find_section(parent, name):
    for s in parent.findall("section"):
        if s.get("name") == name:
            return s
    return ET.SubElement(parent, "section", {"name": name})

def set_attnum(parent, name, val, unit=None):
    attrib = {"name": name, "val": str(val)}
    if unit: attrib["unit"] = unit
    return ET.SubElement(parent, "attnum", attrib)

def set_attstr(parent, name, val):
    return ET.SubElement(parent, "attstr", {"name": name, "val": str(val)})

def set_driver(drivers_sec, slot_idx, module_name):
    s = find_section(drivers_sec, str(slot_idx + 1))
    set_attnum(s, "idx", slot_idx)
    set_attstr(s, "module", module_name)

def main():
    parser = argparse.ArgumentParser(description="Step 1: TORCS two-car straight-line scenario")
    parser.add_argument("--d", type=int, default=80, help="initial gap in meters")
    parser.add_argument("--laps", type=int, default=5, help="number of laps")
    args = parser.parse_args()

    # --- Build XML Tree from Scratch (same as before) ---
    root = ET.Element("params", {"name": "Overtake Scenario"})
    tracks_sec = find_section(root, "Tracks")
    t1 = find_section(tracks_sec, "1")
    set_attstr(t1, "name", DEFAULT_TRACK[0])
    set_attstr(t1, "category", DEFAULT_TRACK[1])
    drivers_sec = find_section(root, "Drivers")
    set_attnum(drivers_sec, "number", 2)
    set_attnum(drivers_sec, "human", 0)
    set_attnum(drivers_sec, "specified", 1)
    set_driver(drivers_sec, 0, "bt")
    set_driver(drivers_sec, 1, "berniw")
    race_manager_sec = find_section(root, "Race Manager")
    set_attnum(race_manager_sec, "laps", args.laps)
    set_attstr(race_manager_sec, "display mode", "normal")
    sg = find_section(root, "Starting Grid")
    set_attnum(sg, "rows", 1)
    set_attnum(sg, "distance between columns", 0, unit="m")
    set_attnum(sg, "offset within a column", args.d, unit="m")
    set_attnum(sg, "initial speed", 170 / 3.6, unit="m/s")
    
    RACEMAN.mkdir(parents=True, exist_ok=True)
    tree = ET.ElementTree(root)
    ET.indent(tree, space="  ", level=0)
    tree.write(OUT_XML, encoding="utf-8", xml_declaration=True)
    print(f"[OK] Wrote diagnostic config to {OUT_XML}")

    # --- THIS IS THE MODIFIED PART ---
    cmd = ["torcs", "-v", "-r", str(OUT_XML)]
    print(f"Running command: {' '.join(cmd)}")
    print("--- Capturing TORCS Process Output ---")

    result = subprocess.run(cmd, capture_output=True, text=True)

    print("\n--- DIAGNOSTIC REPORT ---")
    print(f"TORCS Return Code: {result.returncode}")
    print("\n--- Standard Output (STDOUT) ---")
    print(result.stdout if result.stdout else "[EMPTY]")
    print("\n--- Standard Error (STDERR) ---")
    print(result.stderr if result.stderr else "[EMPTY]")
    print("--- END OF REPORT ---")

if __name__ == "__main__":
    main()