#!/usr/bin/env python3
"""Discover and run all LC29H C++ regressions with address/undefined sanitizers."""
import argparse
import os
from pathlib import Path
import subprocess
import tempfile


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--gps-dir", type=Path, required=True,
                        help="Adafruit GPS 1.9.0 or later source directory")
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[2]
    gps = args.gps_dir.resolve() / "src"
    tests = sorted((root / "extras/tests").rglob("*.cpp"))
    if not tests or not (gps / "Adafruit_GNSS.h").is_file():
        raise RuntimeError("Tests and the shared GPS/GNSS dependency are required")
    sources = [*sorted((root / "src").glob("*.cpp")),
               gps / "Adafruit_NMEA.cpp", gps / "Adafruit_GNSS.cpp"]
    flags = ["-std=c++11", "-Wall", "-Wextra", "-Werror", "-g",
             "-fsanitize=address,undefined", "-fno-sanitize-recover=all",
             "-fno-omit-frame-pointer", "-I" + str(root / "extras/tests/support"),
             "-I" + str(root / "src"), "-I" + str(gps)]
    with tempfile.TemporaryDirectory(prefix="lc29h-tests-") as build:
        for index, test in enumerate(tests):
            binary = str(Path(build) / str(index))
            subprocess.run([os.environ.get("CXX", "g++"), *flags,
                            *map(str, sources), str(test), "-o", binary],
                           check=True, timeout=120)
            subprocess.run([binary], check=True, timeout=30)
    print(f"All {len(tests)} test sources passed.")


if __name__ == "__main__":
    main()
