import os

import numpy as np
import pypatchworkpp

DATA_DIR = os.path.join(os.path.dirname(__file__), "..", "..", "data")


def _read_bin(path):
    return np.fromfile(path, dtype=np.float32).reshape(-1, 4)


def test_module_imports_and_exposes_api():
    assert hasattr(pypatchworkpp, "Parameters")
    assert hasattr(pypatchworkpp, "patchworkpp")


def test_estimate_ground_partitions_all_points():
    params = pypatchworkpp.Parameters()
    pp = pypatchworkpp.patchworkpp(params)

    scan = _read_bin(os.path.join(DATA_DIR, "000000.bin"))
    pp.estimateGround(scan)

    ground = pp.getGround()
    nonground = pp.getNonground()

    assert ground.shape[0] > 0
    assert nonground.shape[0] > 0
    assert ground.shape[0] + nonground.shape[0] <= scan.shape[0]
