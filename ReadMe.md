# GPS and LiDAR Distance Comparison

## Overview

This repository is designed to compare the distances between GPS and LiDAR data, providing a comprehensive analysis of positional discrepancies. The main goal is to analyze how the distances measured by GPS and LiDAR differ under various conditions, and visualize the results through graphical representations.

### Directory Structure

```bash
repo
|- data
    |- dataset_name
        |- targets
            |- OWN_000
                |- ${timestamp}_cla_output.csv
            |- OWN_001
            ...
            |- TARGET_000
            |- TARGET_001
            ...
        |- testset
            |- OWN_000
                |- camera
                    |- ${timestamp}.jpg
                |- fcam_out
                    |- ${timestamp}.json
                |- lidar
                    |- Data
                        |- ${timestamp}.avikus.pcd
                    |- Status
                        |- imu_output.csv
                        |- temperature.csv
                |- metadata
                    |- metadata.txt
                |- oru
                    |- attitude.csv
                    |- position.csv
                    |- metadata.txt
                    |- metadata.txt
                |- lidar.yaml
            |- OWN_001
            ...
            |- TARGET_000
            |- TARGET_001
            ...
|- src
    |- qa_distance.py
    |- qa_inference.py
```

### Data Format

The repository processes data in the following directory structure. The data should be organized under the `data` folder with appropriate files in `targets` and `testset` directories.

1. **targets**:
    - Contains the distance measurement files from different targets (e.g., OWN and TARGET). The distance data is saved as `cla_output.csv` and the target positions in CSV files.
2. **testset**:
    - Includes the actual measurement files for each vehicle (camera images, lidar data, etc.), organized by timestamp.
    - Lidar data in `.avikus.pcd` format, with status files like IMU output and temperature readings.
    - ORU data for attitude, position, and metadata for both ego and target vehicles.

### Running the Script

#### `qa_distance.py`

The core functionality for comparing distances between GPS and LiDAR data is located in `src/qa_distance.py`. This script compares the distance between GPS and LiDAR data and outputs a graph showing the distance comparison and the differences.

#### `qa_inference.py`

The `qa_inference.py` script processes camera images and corresponding target data, draws bounding boxes around detected objects, and calculates the Euclidean distance between object positions. The script then visualizes this information in the form of annotated images and generates a video. It is useful for validating synchronization and object detection in the context of camera and LiDAR data.

### Prerequisites

Before running the scripts, ensure you have the following Python packages installed:

- `geopy`
- `pandas`
- `matplotlib`
- `numpy`
- `statistics`
- `opencv-python`
- `make_video` (custom script)

Install the required packages via pip:

```bash
pip3 install geopy pandas matplotlib numpy statistics opencv-python
```

### How to Use

#### For `qa_distance.py`:

1. **Set up the data directories**:
   Ensure your data is organized as shown in the directory structure, and update the path variables in the script accordingly.

2. **Run the script**:
   Navigate to the `src` folder and run the `qa_distance.py` script with the following command:

   ```bash
   python qa_distance.py --root_path <path_to_data> --case_name <case_name>
   ```

   The script will compare the distances between the ego and target GPS positions and their corresponding LiDAR data.

3. **Outputs**:
   The script generates a CSV file containing the distance comparison results and graphs showing:

   - **Distance Comparison**: GPS vs LiDAR distance.
   - **Distance Difference**: Difference between GPS and LiDAR distances with statistical analysis (mean and standard deviation).
   
   These results are saved as PNG files and a CSV file, named according to the case being tested.

#### For `qa_inference.py`:

1. **Set up the data directories**:
   Ensure your data is organized as shown in the directory structure. The `qa_inference.py` script relies on synchronized camera images, annotations, and target data.

2. **Run the script**:
   Navigate to the `src` folder and run the `qa_inference.py` script with the following command:

   ```bash
   python qa_inference.py --root_path <path_to_data> --case_name <case_name>
   ```

   The script will process the camera images, annotations, and target data, generating bounding boxes around detected objects and calculating the distance for each object. A video is also created from the processed images.

3. **Outputs**:
   - **Processed Images**: The images will be saved with bounding boxes, object class legends, and distance information at the specified output path.
   - **Generated Video**: A video will be created from the processed images and saved as `video.mp4` in the result directory.

### Results

After running `qa_inference.py`, the results include:

- Annotated images with bounding boxes drawn around detected objects.
- A generated video compiled from the processed images, showing the detected objects and their distances.

This script is useful for validating the accuracy of camera data synchronization with target positions, object detection, and distance estimation.

--- 

This updated README now includes the revised details for `qa_inference.py` and covers both scripts, providing full instructions on how to run and use them.