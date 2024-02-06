import argparse
from driving_analysis.data_analyzer import DrivingAnalysis

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--file", required=True, help="Path for the input json file")
    parser.add_argument(
        "--output_file", required=True, help="Path to store the results"
    )
    parser.add_argument(
        "--localisation_max_diff",
        type=float,
        default=0.5,
        help="Maximum allowed distance in metres between expected future coordinates"
        "and actual future coordinates",
    )
    parser.add_argument(
        "max_occluded_frames",
        type=int,
        default=1,
        help="Max number of occluded frames handled by the tracking algorithm",
    )
    args = parser.parse_args()

    analysis = DrivingAnalysis(
        file=args.file,
        output_file=args.output_file,
        localisation_max_diff=args.localisation_max_diff,
        del_track_id_after_missed_frames=args.max_occluded_frames,
    )
    analysis.run()
