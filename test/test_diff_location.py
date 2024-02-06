import os
import shutil

import pytest

from driving_analysis.data_analyzer import DrivingAnalysis


@pytest.fixture
def temp_dir():
    temp_path = "tmp"
    os.makedirs(temp_path, exist_ok=True)
    yield temp_path
    shutil.rmtree(temp_path)


def test_diff_location(temp_dir):
    output_file = os.path.join(temp_dir, "results.json")
    file = os.path.join("test", "data", "diff_location_test.json")
    da = DrivingAnalysis(file, output_file=output_file)
    da.diff_location(1, 0)
    da.diff_location(2, 1)
    assert 1 not in da.anomalies["localisation"]
    assert 2 in da.anomalies["localisation"]
