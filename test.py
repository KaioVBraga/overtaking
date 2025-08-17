#!/usr/bin/env python3
import argparse, os, subprocess, sys, xml.etree.ElementTree as ET
from pathlib import Path

HOME = Path.home()
RACEMAN = HOME / ".torcs" / "config" / "raceman"
SRC_XML = RACEMAN / "quickrace.xml"           # created by first run of TORCS
OUT_XML = RACEMAN / "overtake_step1.xml"

# Default to built-in road track; auto-switch to your custom straight track if present
DEFAULT_TRACK = ("g-track-1", "road")         # safe built-in choice
CUSTOM_TRACK = ("straight-2800x15", "road")   # what you create in step 3
VALID_D = {80,100,120,140,160,180,200,220,240,260,280,300}

def need_first_run():
    if not SRC_XML.exists():
        print("It looks like ~/.torcs/config/raceman/quickrace.xml is missing.\n"
              "Run `torcs` once, exit, then re-run this script.", file=sys.stderr)
        sys.exit(1)

def pick_track():
    # Detect if a user/system track folder exists with our custom name
    candidates = [
        HOME / ".torcs" / "tracks" / "road" / CUSTOM_TRACK[0],
        Path("/usr/local/share/games/torcs/tracks/road") / CUSTOM_TRACK[0],
        Path("/usr/share/games/torcs/tracks/road") / CUSTOM_TRACK[0],
    ]
    for p in candidates:
        if p.exists():
            return CUSTOM_TRACK   # (name, category)
    return DEFAULT_TRACK

def find_section(parent, name):
    for s in parent.findall("section"):
        if s.get("name") == name:
            return s
    s = ET.SubElement(parent, "section", {"name": name})
    return s

def set_attnum(parent, name, val, unit=None):
    # find or create <attnum name="..." val="..."/>
    for a in parent.findall("attnum"):
        if a.get("name") == name:
            a.set("val", str(val))
            if unit: a.set("unit", unit)
            return a
    attrib = {"name": name, "val": str(val)}
    if unit: attrib["unit"] = unit
    return ET.SubElement(parent, "attnum", attrib)

def set_attstr(parent, name, val):
    for a in parent.findall("attstr"):
        if a.get("name") == name:
            a.set("val", str(val)); return a
    return ET.SubElement(parent, "attstr", {"name": name, "val": str(val)})

def set_driver(drivers_sec, slot_idx, module_name):
    s = find_section(drivers_sec, str(slot_idx + 1))
    set_attnum(s, "idx", slot_idx)
    set_attstr(s, "module", module_name)

def main():
    parser = argparse.ArgumentParser(description="Step 1: TORCS two-car straight-line scenario")
    parser.add_argument("--d", type=int, required=True,
                        help="initial gap in meters (choose from {80,100,...,300})")
    parser.add_argument("--laps", type=int, default=1, help="number of laps (default 1)")
    parser.add_argument("--show", action="store_true",
                        help="print the generated XML path and exit without launching TORCS")
    args = parser.parse_args()

    if args.d not in VALID_D:
        print(f"--d must be in {sorted(VALID_D)}", file=sys.stderr); sys.exit(2)

    need_first_run()

    tree = ET.parse(SRC_XML)
    root = tree.getroot()  # <params name="Quick Race"> at top

    # === Track selection ===
    tracks_sec = find_section(root, "Tracks")
    t1 = find_section(tracks_sec, "1")
    name, category = pick_track()
    set_attstr(t1, "name", name)
    set_attstr(t1, "category", category)

    # === Drivers (two TORCS bots that use car1-trb1 class) ===
    drivers_sec = find_section(root, "Drivers")
    set_driver(drivers_sec, 0, "bt")      # opponent 1
    set_driver(drivers_sec, 1, "berniw")  # opponent 2

    # === Race length & mode (keep UI visible) ===
    set_attnum(root, "distance", 0, unit="km")  # distance=0 => use laps
    set_attnum(root, "laps", args.laps)
    set_attstr(root, "display mode", "normal")

    # === Starting Grid: single-file, longitudinal offset d, initial speed 170 km/h ===
    sg = find_section(root, "Starting Grid")
    set_attnum(sg, "rows", 1)                                 # one car per row (single file)
    set_attnum(sg, "distance between columns", 0, unit="m")   # no side-by-side columns
    set_attnum(sg, "offset within a column", args.d, unit="m")# spacing along the straight
    set_attnum(sg, "initial speed", 170/3.6, unit="m/s")      # ≈47.22 m/s

    # Write new scenario file
    tree.write(OUT_XML, encoding="utf-8", xml_declaration=True)
    print(f"[OK] Wrote {OUT_XML}")

    if args.show:
        return

    # Launch TORCS on this scenario (you get the GUI so you can watch)
    # Tip: -nofuel -nodamage are useful while debugging
    cmd = ["torcs", "-r", str(OUT_XML)]
    print("Launching:", " ".join(cmd))
    subprocess.run(cmd)

if __name__ == "__main__":
    main()
