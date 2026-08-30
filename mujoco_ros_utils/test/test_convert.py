#!/usr/bin/env python3

"""convert_dae_to_stl converts a dae to a stl which mujoco can load."""

import os
import shutil
import struct
import sys
import tempfile
import unittest

PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(PKG_DIR, "scripts"))

from convert import convert_dae_to_stl  # noqa: E402

DAE_PATH = os.path.join(PKG_DIR, "test", "data", "main_body.dae")


class TestConvertDaeToStl(unittest.TestCase):
    def test_dae_is_converted_to_binary_stl(self):
        workdir = tempfile.mkdtemp()
        try:
            stl_path = os.path.join(workdir, "main_body.stl")
            convert_dae_to_stl(DAE_PATH, stl_path)
            self.assertTrue(os.path.isfile(stl_path), "stl is not generated from dae")

            with open(stl_path, "rb") as f:
                data = f.read()
            # binary stl: 80 bytes of header, face num, and 50 bytes per face
            face_num = struct.unpack("<I", data[80:84])[0]
            self.assertGreater(face_num, 0, "converted stl has no face")
            self.assertEqual(len(data), 84 + 50 * face_num, "converted stl is not a binary stl")
        finally:
            shutil.rmtree(workdir)


if __name__ == "__main__":
    unittest.main()
