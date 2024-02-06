# Installation Guide

### Setting up the environment, install requirements and python package:

            python3.10 -m venv .venv
            pip install -r requirements.txt
            pip install -e .

### Running the main.py

            python main.py --file /path/to/input/driving_data.json \
                        --output_file /path/to/output/results.json \
                        --localisation_max_diff YOUR_LOCALISATION_MAX_DIFF_VALUE \
                        --max_occluded_frames YOUR_MAX_OCCLUDED_FRAMES_VALUE

<br>

## Instructions to create a Docker image and run the software in a containerized environment:

### 1. Create Docker Image:

            docker build -t driving-analysis .

### 2. To run this software as a container how should modify the following paths of the CL instructions below:

<br>

- _`/path/to/this/folder/driving_data`_: insert the path to the folder containing `driving_data.json`
- _`/path/to/desired/output/folder`_: Specify the directory to save the output JSON file
- _`--localisation_max_diff`_: Specify the desired maximum distance in metres.
- _`--max_occluded_frames`_: Set the maximum number of occluded frames allowed by the tracking algorithm.

            docker run -v /path/to/this/folder/driving_data:/data \
                    -v /path/to/desired/output/folder:/results \
                    driving-analysis --file /data/driving_data.json \
                    --output_file /results/results.json \
                    --localisation_max_diff <max-diff-value> \
                    --max_occluded_frames <max-occluded-frames-value>
