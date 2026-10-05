#!/usr/bin/env python3
"""Static repository consistency checks; this is not an Ada/native build."""
from pathlib import Path
import os
import re
import sys
import tomllib

ROOT = Path(__file__).resolve().parent.parent
CORE_PIN = "4da9d35ea21e1b2efe96296243ea668b488c6326"


def check(condition: bool, message: str) -> None:
    if not condition:
        raise ValueError(message)


def bridge_not_vendored() -> None:
    for directory, children, files in os.walk(ROOT):
        children[:] = [n for n in children if n not in {".git", "alire", "obj"}]
        check("opencv_core_module_bridge.hpp" not in files, "do not vendor Core's bridge")


def main() -> None:
    manifests = [tomllib.loads((ROOT / p).read_text()) for p in
                 ("alire.toml", "tests/alire.toml", "examples/alire.toml")]
    production = manifests[0]
    dependencies = {k for group in production["depends-on"] for k in group}
    check({"opencv_core", "opencv", "pkg_config"} <= dependencies,
          "missing production dependency")
    check(not dependencies & {"opencv_features", "opencv_imgproc", "opencv_calib3d", "aunit"},
          "production dependency boundary changed")
    commits = [m["pins"][-1]["opencv_core"]["commit"] for m in manifests]
    check(commits == [CORE_PIN, CORE_PIN, CORE_PIN], "Core pins differ from bootstrap pin")

    header = (ROOT / "cpp/opencv_video_shim.h").read_text()
    ada = (ROOT / "src/internal/opencv-video-internal-c_api.ads").read_text()
    cpp = (ROOT / "cpp/opencv_video_shim.cpp").read_text()
    declared = set(re.findall(r"\b(opencv_video_\w+)\s*\(", header))
    imported = set(re.findall(r'External_Name\s*=>\s*"(opencv_video_\w+)"', ada))
    check(declared == imported, f"C/Ada import mismatch: {declared ^ imported}")
    check(all(re.search(r"\b" + re.escape(name) + r"\s*\(", cpp) for name in declared),
          "missing C++ export")

    bridge_not_vendored()
    check(not list((ROOT / "src").rglob("opencv.ads")), "do not redeclare Core's root package")
    for path in list((ROOT / "src").rglob("*.ads")) + list((ROOT / "src").rglob("*.adb")):
        check("External_Name" not in path.read_text() or "/internal/" in path.as_posix(),
              f"C import leaked into public Ada: {path}")

    tests = (ROOT / "tests/src/video_tests.adb").read_text()
    registrations = re.findall(r"Result\.Add_Test\s*\(Caller\.Create", tests)
    check(len(registrations) == 22, "update documented AUnit inventory when changing tests")

    configure = (ROOT / "scripts/configure_opencv.sh").read_text()
    check("opencv2/video/tracking.hpp" in configure and "libopencv_video" in configure,
          "native video module/header checks are missing")
    check("backend=video" in configure, "native backend metadata is missing")

    workflows = ROOT / ".github/workflows"
    cross = (workflows / "cross-platform.yml").read_text()
    windows = (workflows / "windows-post-merge.yml").read_text()
    compat = (workflows / "opencv-compatibility.yml").read_text()
    check("pull_request:" in cross and "linux-sanitizers:" in cross and "macos:" in cross,
          "cross-platform workflow topology changed")
    check("pull_request:" not in windows and "workflow_dispatch:" not in windows,
          "Windows must remain post-merge-only")
    check("workflow_dispatch:" in compat and "pull_request:" not in compat,
          "compatibility matrix must remain manual-only")

    print(f"PASS: manifests, Core pin {CORE_PIN[:12]}, {len(declared)} ABI declarations/imports, "
          f"22 AUnit registrations, Core ownership, video backend, CI topology")


if __name__ == "__main__":
    try:
        main()
    except (OSError, KeyError, ValueError) as error:
        print(f"FAIL: {error}", file=sys.stderr)
        sys.exit(1)
