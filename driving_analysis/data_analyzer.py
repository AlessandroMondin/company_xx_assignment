import json
import math
import os


class DrivingAnalysis:
    def __init__(
        self,
        file: str,
        output_file: str,
        localisation_max_diff: float = 3.0,
        del_track_id_after_missed_frames: int = 0,
    ) -> None:
        """Driving analysis class to perform anomaly detection
            on localisation and perception data.

        Args:
            file (str): path to json file
            output_file (str): path that indicates where to store the json file that
                                contains the results of the analysis.
            localisation_max_diff (float): the maximum difference in meters between the
                                            next expected location and actual next
                                            location.
            del_track_id_after_missed_frames (int): number of frames for which a track_id
                                                    needs to not be detected before being
                                                    deleted. This argument is usefull when
                                                    checking the consistency of the
                                                    currently tracked objects. In fact if
                                                    the input data is stored by using a
                                                    tracking algorithm that handles object
                                                    occlusion, then we want to consider it
                                                    to correcly record anomalies in
                                                    perception system.
        """

        if os.path.exists(file) is False:
            raise FileNotFoundError(f"The path {file} does not exist.")
        with open(file, mode="r") as f:
            self.driving_info = json.load(f)

        if os.path.exists(os.path.dirname(output_file)) is False:
            raise FileNotFoundError(
                f"Directory {os.path.dirname(output_file)} does not exists."
            )
        self.output_file = output_file

        # dictionary containing all the detected anomalies
        self.anomalies = {
            "localisation": {},
            "perception": {"raw": {}, "summary": {}},
        }

        self.currently_tracked_objects = {}

        self.localisation_max_diff = localisation_max_diff

        self.del_track_id_after_missed_frames = del_track_id_after_missed_frames

    @property
    def tot_localisation_obs(self):
        return len(self.driving_info["ego"]["scene_id"])

    @property
    def tot_perception_obs(self):
        return len(self.driving_info["obs"])

    @property
    def fps(self):
        return self.driving_info["fps"]

    def run(self):
        total_iteration = max(self.tot_localisation_obs, self.tot_perception_obs)
        for i in range(1, total_iteration):
            if i < self.tot_localisation_obs:
                self.diff_location(i, i - 1)

            if i < self.tot_perception_obs:
                # the iteration starts at 1, but here we start from the
                # first frame, that's why i - 1
                self.tracked_object_type_consistency(i - 1)

        with open(self.output_file, mode="w") as f:
            json.dump(self.anomalies, f)

    def diff_location(self, scene_timestep, scene_previous_timestep):
        """
        Stores all the frames where the difference between the actual coordinates and the
        expected coordinates exceeds a given threshold. The expected coordinates are
        computed in the following way: since `distance = speed * time`, we can estimate
        the movement across the x and y axis using the cosine and sine of the yaw. By
        adding together the initial coordinates and the expected movement across each
        axis, then we can estimate the future position of the ego vehicle.

        Args:
            scene_timestep (int): index of the scene at the current timestep.
            scene_previous_timestep (int): index of the scene at the previous timestep.
        """
        # x, y, speed, yaw of the previous frame.
        prev_x = self.driving_info["ego"]["x"][scene_previous_timestep]
        prev_y = self.driving_info["ego"]["y"][scene_previous_timestep]
        prev_speed = self.driving_info["ego"]["speed"][scene_previous_timestep]
        prev_yaw = self.driving_info["ego"]["yaw"][scene_previous_timestep]

        # predicts the expected movement based on speed and yaw
        expected_change_x = prev_speed * math.cos(prev_yaw) * (1 / self.fps)
        expected_change_y = prev_speed * math.sin(prev_yaw) * (1 / self.fps)

        # calculates the expected next position by summing the previous position
        # and the expected movement.
        expected_curr_x = prev_x + expected_change_x
        expected_curr_y = prev_y + expected_change_y

        # Actual "next" coordinates
        curr_x = self.driving_info["ego"]["x"][scene_timestep]
        curr_y = self.driving_info["ego"]["y"][scene_timestep]

        # calculate the difference between actual and expected coordinates
        diff_x = curr_x - expected_curr_x
        diff_y = curr_y - expected_curr_y

        # calculate the direct (diagonal) distance using Pythagorean theorem
        direct_distance_diff = math.sqrt(diff_x**2 + diff_y**2)

        # if the actual distance exceeds the maximum distance set as a
        # class argument, then we store the current scene within the anonomalies.
        if direct_distance_diff > self.localisation_max_diff:
            self.anomalies["localisation"][scene_timestep] = round(
                direct_distance_diff, 2
            )

    def tracked_object_type_consistency(self, scene_id):
        """
        Checks the consistency of the "object class" objects currently being tracked.
        This method internally stores in a hash-map the track_ids and the corresponding
        classes (i.e. "pededestrian", "car", etc) and then controls that, whether a
        track_id is both present in the hash-map and in the current scene, then their
        object type must be the same. If there is no consistency between the object class
        in the hash-map and in the scene, we store the scene and the object classes (old
        and new) within the anomalies.
        This method handles object occlusion by allowing to keep track of 'track_ids'
        stored in the tracking hashmap a specified number of frames even if they do not
        show up in the current scenes. The instance argument
        `del_track_id_after_missed_frames` controls the maximum number of occluded frames
        before deleting a track_id.

        Args:
            scene_id (int): index of the scene at the current timestep.
        """
        # Some frames do not contain any data, we store them within the
        # anomalies.
        if str(scene_id) not in self.driving_info["obs"].keys():
            self.anomalies["perception"]["raw"][str(scene_id)] = None
            if "empty_frames" not in self.anomalies["perception"]["summary"]:
                self.anomalies["perception"]["summary"]["empty_frames"] = 1
            else:
                self.anomalies["perception"]["summary"]["empty_frames"] += 1
            return

        # all the perception data recorded in the current scene
        scene = self.driving_info["obs"][str(scene_id)]

        # set of unique tracking ids currently being tracked
        current_tracks_id = set(self.currently_tracked_objects.keys())
        # set of unique ids tracked in the current scene
        scene_tracks_id = set(scene["scene_id"])

        # track_ids currently tracked that were not present in the current frame
        lost_tracked_id = current_tracks_id - scene_tracks_id
        for lost_track_id in lost_tracked_id:
            # the number of frames that a track_id has been not detected
            self.currently_tracked_objects[lost_track_id]["undetected_frame_count"] += 1
            count_lost_track_id = self.currently_tracked_objects[lost_track_id][
                "undetected_frame_count"
            ]

            # if a given tracking_id has not been detected within the last
            # `self.del_track_id_after_missed_frames`, then its track_id is
            # set free (deleted).
            if count_lost_track_id > self.del_track_id_after_missed_frames:
                del self.currently_tracked_objects[lost_track_id]

        # for each distinct tracking_id, detected in this given scene, we check if the
        # tracking_id is already tracked by the `currently_tracked_objects` hash-map.
        # If a given `track_id` is not being tracked, it is added to the data-structure,
        # otherwise it is checked if the if `object_type` tracked in the  current frame
        # is the same as the `object_type` stored in the hash-map.s

        for i in range(len(scene["scene_id"])):
            track_id = self.driving_info["obs"][str(scene_id)]["scene_id"][i]
            object_type = self.driving_info["obs"][str(scene_id)]["type"][i]
            # if a given track id is not being currently tracked, we add it to
            # the hash-map.
            if track_id not in self.currently_tracked_objects:
                self.currently_tracked_objects[track_id] = {
                    "object_type": object_type,
                    "undetected_frame_count": 0,
                }
            # if a given track id is tracked but the object type is different from the
            # one detected within the scene, than we have identified an anomaly.
            elif self.currently_tracked_objects[track_id]["object_type"] != object_type:
                previous_object_type = self.currently_tracked_objects[track_id][
                    "object_type"
                ]
                # we do not consider as anomalies cases where the detected class was
                # unknown since it might indicate that the detected object was very
                #  distant and the confidence score of the detection was low.
                if "UNKNOWN" in [object_type, previous_object_type]:
                    continue
                # stores the raw data
                self.anomalies["perception"]["raw"][str(scene_id)] = {
                    "track_id": track_id,
                    "previous_object_type": previous_object_type,
                    "new_object_type": object_type,
                }
                # stores the summary data.
                key = f"{previous_object_type.lower()}_2_{object_type.lower()}"
                summary = self.anomalies["perception"]["summary"]
                if key not in summary:
                    summary[key] = 1
                else:
                    summary[key] += 1
