#!/usr/bin/env python3
"""Exercise the actual configure script in isolated temporary installations."""
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parent.parent


class ConfigurationTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        (self.root / "scripts").mkdir()
        for name in ("configure_opencv.sh", "build_opencv_shim.sh"):
            shutil.copy2(ROOT / "scripts" / name, self.root / "scripts" / name)
        self.include = self.root / "include"
        (self.include / "opencv2/video").mkdir(parents=True)
        (self.include / "opencv2/video/tracking.hpp").touch()
        self.lib = self.root / "lib"
        self.lib.mkdir()
        (self.lib / "libopencv_video.so").touch()
        self.core = self.root / "core"
        (self.core / "cpp").mkdir(parents=True)
        (self.core / "cpp/opencv_core_module_bridge.hpp").touch()
        self.bin = self.root / "bin"
        self.bin.mkdir()
        self.executable("uname", "echo Linux\n")
        self.executable("pkg-config", """case "$1" in
--exists) test "$2" = "${TEST_PACKAGE:-opencv4}" ;;
--modversion) echo "${TEST_VERSION:-4.10.0}" ;;
--variable=includedir) echo "$TEST_INCLUDE" ;;
--variable=libdir) echo "$TEST_LIB" ;;
*) exit 1 ;;
esac
""")
        self.env = dict(os.environ, PATH=f"{self.bin}:{os.environ['PATH']}",
                        PKG_CONFIG=str(self.bin / "pkg-config"),
                        TEST_INCLUDE=str(self.include), TEST_LIB=str(self.lib),
                        OPENCV_CORE_ALIRE_PREFIX=str(self.core))

    def executable(self, name, body):
        path = self.bin / name
        path.write_text("#!/bin/sh\n" + body)
        path.chmod(0o755)

    def configure(self, success):
        result = subprocess.run(["sh", "scripts/configure_opencv.sh"], cwd=self.root,
                                env=self.env, text=True, capture_output=True)
        self.assertEqual(result.returncode == 0, success, result.stdout + result.stderr)
        output = self.root / "config/opencv_video_install.gpr"
        self.assertEqual(output.exists(), success)
        if success:
            self.assertIn('Native_Backend := "video";', output.read_text())
            self.assertIn('Shim_Build_Kind', output.read_text())

    def test_supported_versions(self):
        for version in ("4.1.0", "4.10.0", "5.0.0"):
            with self.subTest(version=version):
                self.env["TEST_VERSION"] = version
                self.configure(True)

    def test_unsupported_versions(self):
        for version in ("4.0.0", "3.4.0", "5.1.0", "6.0.0", "4.10.0-dev"):
            with self.subTest(version=version):
                self.env["TEST_VERSION"] = version
                self.configure(False)

    def test_missing_core_prefix(self):
        self.env.pop("OPENCV_CORE_ALIRE_PREFIX")
        self.configure(False)

    def test_missing_bridge(self):
        (self.core / "cpp/opencv_core_module_bridge.hpp").unlink()
        self.configure(False)

    def test_missing_video_header(self):
        (self.include / "opencv2/video/tracking.hpp").unlink()
        self.configure(False)

    def test_missing_video_library(self):
        (self.lib / "libopencv_video.so").unlink()
        self.configure(False)

    def test_opencv5_metadata(self):
        self.env.update(TEST_PACKAGE="opencv5", TEST_VERSION="5.0.0")
        self.configure(True)


if __name__ == "__main__":
    unittest.main()