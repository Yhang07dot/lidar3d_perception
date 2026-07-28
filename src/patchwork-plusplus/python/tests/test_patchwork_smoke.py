# python/tests/test_patchwork_smoke.py
import os

import numpy as np
import pypatchworkpp

DATA_DIR = os.path.join(os.path.dirname(__file__), "..", "..", "data")


def _read_bin(path):
    return np.fromfile(path, dtype=np.float32).reshape(-1, 4)


def test_classic_module_exposes_api():
    assert hasattr(pypatchworkpp, "PatchworkParams")
    assert hasattr(pypatchworkpp, "patchwork")


def test_classic_estimate_ground_partitions_all_points():
    params = pypatchworkpp.PatchworkParams()
    pw = pypatchworkpp.patchwork(params)

    scan = _read_bin(os.path.join(DATA_DIR, "000000.bin"))
    pw.estimateGround(scan)

    ground = pw.getGround()
    nonground = pw.getNonground()

    assert ground.shape[0] > 0
    assert nonground.shape[0] > 0
    assert ground.shape[0] + nonground.shape[0] <= scan.shape[0]
