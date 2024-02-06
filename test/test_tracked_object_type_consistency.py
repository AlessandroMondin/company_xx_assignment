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


# This test checks that any time the object class associated to a given track_id
# changes over frames, it is added to the anomalies. In fact this suggests problems
# within the perception system.s
def test_tracked_object_type_consistency(temp_dir):
    outdir = os.path.join(temp_dir, "results.json")
    file = os.path.join("test", "data", "tracked_objects_test.json")

    da = DrivingAnalysis(file, output_file=outdir)

    # Keeps track of the object in the first frame
    da.tracked_object_type_consistency(0)
    # Since the ground truth json file contains two object in the first frame,
    # we check that two object are detected
    assert len(da.currently_tracked_objects) == 2

    # Updates the tracked objects with the second frame.
    da.tracked_object_type_consistency(1)

    # In the ground truth, the track_id '1' change his detection
    # from car to truck between the first and second frame, and
    # we add id to the tracked anomalies.s
    assert "1" in da.anomalies["perception"]["raw"]
    assert da.anomalies["perception"]["summary"] == {"car_2_truck": 1}


# This test checks the algorithm's ability to handle objects occlusion.
def test_occlusion_handling(temp_dir):
    outdir = os.path.join(temp_dir, "results.json")
    file = os.path.join("test", "data", "occlusion_handling.json")

    # Initialize the DrivingAnalysis class and specified the number of occluded
    # frames used before removing a track_id.
    da = DrivingAnalysis(
        file=file, output_file=outdir, del_track_id_after_missed_frames=2
    )

    # Processes the first and second frame. In the grount truth data, in frame 0
    # the object is detected, while in frame 1 and 2 the object is not detected.
    # However since we set the `del_track_id_after_missed_frames` to 2 frames,
    # the object must not be removed from the `currently_tracked_objects` attribute
    da.tracked_object_type_consistency(0)
    da.tracked_object_type_consistency(1)
    assert 1 in da.currently_tracked_objects
    da.tracked_object_type_consistency(2)
    assert 1 in da.currently_tracked_objects

    # Reinitialize the instance. Now the `del_track_id_after_missed_frames` is
    # set to 1 and therefore at the second occluded frame, the object should be
    # removed from the `currently_tracked_objects` attribute.
    da = DrivingAnalysis(
        file=file, output_file=outdir, del_track_id_after_missed_frames=1
    )

    da.tracked_object_type_consistency(0)
    da.tracked_object_type_consistency(1)
    assert 1 in da.currently_tracked_objects
    da.tracked_object_type_consistency(2)

    # At the second frame that the object is occluded, we need to check the it is
    # remove from the object currently being tracked.
    assert 1 not in da.currently_tracked_objects
