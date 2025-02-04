### README.txt

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

The core functionality is located in `src/qa_distance.py`. This script compares the distance between GPS and LiDAR data and outputs a graph showing the distance comparison and the differences.

### Prerequisites

Before running the script, ensure you have the following Python packages installed:

- `geopy`
- `pandas`
- `matplotlib`
- `numpy`
- `statistics`

Install them via pip:

```bash
pip3 install geopy pandas matplotlib numpy statistics
```

### How to Use

1. **Set up the data directories**:
   Ensure your data is organized as shown in the directory structure and the path variables in the script are updated correctly.

2. **Run the script**:
   Navigate to the `src` folder and run the `qa_distance.py` script:

   ```bash
   python qa_distance.py --root_path <path_to_data> --case_name <case_name>
   ```

   The script will compare the distances between the ego and target GPS positions and their corresponding LiDAR data.

### Outputs

The script generates a CSV file containing the following columns:

- `idx`: Index for each data point.
- `ego_gps_timestamp`: Timestamp for the ego GPS data.
- `ego_lat`, `ego_lon`: Latitude and longitude for the ego boat.
- `tgt_gps_timestamp`: Timestamp for the target GPS data.
- `tgt_lat`, `tgt_lon`: Latitude and longitude for the target boat.
- `lidar_timestamp`: Timestamp for the nearest LiDAR data.
- `lidar_path`: Path to the corresponding LiDAR file.

Additionally, the script produces graphs showing:

1. **Distance Comparison**: GPS vs LiDAR distance.
2. **Distance Difference**: Difference between GPS and LiDAR distances with statistical analysis (mean and standard deviation).

These results are saved as PNG files, named according to the case being tested.
