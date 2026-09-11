#!/usr/bin/env python3
"""Verify vendored robot/positioner STLs are unmodified ROS-Industrial copies.

The browser never decimates these files. Visual meshes must match the
ros-industrial/motoman ``noetic-devel`` visual STLs byte-for-byte.
Collision hulls are a separate, coarser set used only if collision parsing
is enabled — they must not be substituted for the visual CAD.

Usage:
  python3 tools/verify_vendor_meshes.py
  python3 tools/verify_vendor_meshes.py --fetch-upstream
"""

from __future__ import annotations

import argparse
import hashlib
import sys
import urllib.request
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
MODELS = ROOT / "frontend" / "public" / "models"
RAW = "https://raw.githubusercontent.com/ros-industrial/motoman/noetic-devel"

# SHA-256 of the unmodified ROS-Industrial files (noetic-devel).
EXPECTED: dict[str, tuple[str, str]] = {
    # local relative path -> (sha256, upstream path under the motoman repo)
    "ar2010/visual/base_link.stl": (
        "a45502cd0e7556af8c2faa3978856fe00ee72259bfa395aefe739db29d51553f",
        "motoman_ar2010_support/meshes/visual/base_link.stl",
    ),
    "ar2010/visual/link_1_s.stl": (
        "330bbf49063592b008008ec927f814ec0c9e315e04785aaa1bc039995a72dec6",
        "motoman_ar2010_support/meshes/visual/link_1_s.stl",
    ),
    "ar2010/visual/link_2_l.stl": (
        "c64c9f54cffb42163f4c937c251b2762a57d82ad2630ff60ef2ce36963fa8f97",
        "motoman_ar2010_support/meshes/visual/link_2_l.stl",
    ),
    "ar2010/visual/link_3_u.stl": (
        "77a24508cc7a9c1042675daff07020a49d6c0bd41edbcd0a414422ba6aab9424",
        "motoman_ar2010_support/meshes/visual/link_3_u.stl",
    ),
    "ar2010/visual/link_4_r.stl": (
        "9ee26141691fb94c1d4b10900b46e534f6c629311bede9d40c2c2edc3a10b7cc",
        "motoman_ar2010_support/meshes/visual/link_4_r.stl",
    ),
    "ar2010/visual/link_5_b.stl": (
        "8d6d2315d556c214711681b619855cf6b61c44bd9d13b0a7ec92600c7f7128c2",
        "motoman_ar2010_support/meshes/visual/link_5_b.stl",
    ),
    "ar2010/visual/link_6_t.stl": (
        "ecd700ab1b91a99e8f2c4013062fc9c8e408f6fe4f76b4e646eb640a0a745481",
        "motoman_ar2010_support/meshes/visual/link_6_t.stl",
    ),
    "ar2010/collision/base_link.stl": (
        "f42eb9691c9d85eaa7a4d342394d7347522935a3af1c13a2d96a3d7090131e43",
        "motoman_ar2010_support/meshes/collision/base_link.stl",
    ),
    "ar2010/collision/link_1_s.stl": (
        "0b8b39dd7103276455810431ec7c3b3a940252310fb3c2c85902fd6c4677ef43",
        "motoman_ar2010_support/meshes/collision/link_1_s.stl",
    ),
    "ar2010/collision/link_2_l.stl": (
        "8ccbb8537e5cd3fc642a3f4d3733bfc8857f7fd660b729f2b5eea0387f979a63",
        "motoman_ar2010_support/meshes/collision/link_2_l.stl",
    ),
    "ar2010/collision/link_3_u.stl": (
        "18b71b7218d0f61e3aeeb387891ddc53e154bffebdcf364c46d6dc1bb95126ad",
        "motoman_ar2010_support/meshes/collision/link_3_u.stl",
    ),
    "ar2010/collision/link_4_r.stl": (
        "8ec4582aa4178772c03cc247995ef4c2772c3bae567232f32709c965dbd26a23",
        "motoman_ar2010_support/meshes/collision/link_4_r.stl",
    ),
    "ar2010/collision/link_5_b.stl": (
        "89dc3d2c71768c9a727e356f1771bd6c59839d2cb92003ff432ce326e7f5e09d",
        "motoman_ar2010_support/meshes/collision/link_5_b.stl",
    ),
    "ar2010/collision/link_6_t.stl": (
        "63cda826e2f4c92bc08df008f355ea79cb31fe558d2410d732445a37c590ae2c",
        "motoman_ar2010_support/meshes/collision/link_6_t.stl",
    ),
    "motopos_d500/visual/base_link.stl": (
        "422289f11b2dcef6d77b9679b9359efc7ba3648dd38ca86098491bd412d7ddd6",
        "motoman_motopos_d500_support/meshes/motopos_d500/visual/base_link.stl",
    ),
    "motopos_d500/visual/link_1.stl": (
        "3822d4669a89ac2c65aca0a956c4a0a75f0df95ee311ce55d0acfe27f8697296",
        "motoman_motopos_d500_support/meshes/motopos_d500/visual/link_1.stl",
    ),
    "motopos_d500/visual/link_2.stl": (
        "ef8947f40d99ac3eb752cc71c709c51e5aa0d5759bc69674e8922290536c52ff",
        "motoman_motopos_d500_support/meshes/motopos_d500/visual/link_2.stl",
    ),
    "motopos_d500/collision/base_link.stl": (
        "eb4127d88ddfb810cd3a8a14e0cc4f949c5f3592e24bbca0595e125036bcdf8d",
        "motoman_motopos_d500_support/meshes/motopos_d500/collision/base_link.stl",
    ),
    "motopos_d500/collision/link_1.stl": (
        "36aac4cdd4cf02d83c0fa283d5f9b2c34a77d75e9cb2c833e12786ba58e18462",
        "motoman_motopos_d500_support/meshes/motopos_d500/collision/link_1.stl",
    ),
    "motopos_d500/collision/link_2.stl": (
        "d5cb981f809fb3fd29bf6b9199464509b905010f1856530d8e0b7b372730c7a2",
        "motoman_motopos_d500_support/meshes/motopos_d500/collision/link_2.stl",
    ),
}


def sha256(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument(
        "--fetch-upstream",
        action="store_true",
        help="Also download each file from GitHub and compare bytes.",
    )
    args = ap.parse_args()

    failed = 0
    for rel, (digest, upstream) in EXPECTED.items():
        path = MODELS / rel
        if not path.is_file():
            print(f"MISSING {rel}")
            failed += 1
            continue
        data = path.read_bytes()
        got = sha256(data)
        if got != digest:
            print(f"HASH MISMATCH {rel}\n  expected {digest}\n  got      {got}")
            failed += 1
            continue
        if args.fetch_upstream:
            url = f"{RAW}/{upstream}"
            with urllib.request.urlopen(url) as resp:
                remote = resp.read()
            if remote != data:
                print(f"UPSTREAM MISMATCH {rel} ({url})")
                failed += 1
                continue
        kind = "visual" if "/visual/" in rel.replace("\\", "/") else "collision"
        print(f"ok  {kind:9} {rel}  ({len(data)} bytes)")

    if failed:
        print(f"\n{failed} file(s) failed")
        return 1
    print("\nAll vendored meshes match the pinned ROS-Industrial originals.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
