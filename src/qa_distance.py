import argparse
import pandas as pd
import math
import os
import sys
import csv
from geopy import distance

from datetime import datetime
import numpy as np
import pandas as pd
import numpy as np
import pdb
from datetime import datetime, timezone, timedelta
import matplotlib.pyplot as plt
import statistics

#### Global Variables
Epsilon = 10000  # (m)
THRESHOLD_EGO_TGT_GPS = 1000
DISTANCE_THRESHOLD = 4

class TrackIdError(Exception):
    pass


def draw_graph(idx: list, gps_distance: list, lidar_distance: list, tgt_relative_bearing_list: list, ego_hdg_list: list, tgt_hdg_list: list, own_relative_bearing_list: list, result_path: str):
    # Calculate the difference (skip None values)
    filtered_idx = []
    distance_difference = []
    valid_distance_diff = []
    in_tgt_relative_bearing_list, in_ego_hdg_delta_list, in_tgt_hdg_delta_list, in_own_relative_bearing_list, in_distance_list = [], [], [], [], []
    out_tgt_relative_bearing_list, out_ego_hdg_delta_list, out_tgt_hdg_delta_list, out_own_relative_bearing_list, out_distance_list = [], [], [], [], []

    old_ego_hdg, old_tgt_hdg = -math.inf, -math.inf

    for i, (gps, lidar, tgt_rel_bearing, ego_hdg, tgt_hdg, own_rel_bearing) in enumerate(zip(gps_distance, lidar_distance, tgt_relative_bearing_list, ego_hdg_list, tgt_hdg_list, own_relative_bearing_list)):
        if (
            gps is not None
            and lidar is not None
            and not math.isnan(gps)
            and not math.isnan(lidar)
        ):
            filtered_idx.append(idx[i])  # Add valid idx
            distance_difference.append(gps - lidar)
            valid_distance_diff.append(gps - lidar)
            # valid_distance_diff_tgt_rel_bearing.append(abs(math.sin(tgt_rel_bearing)))
            if (abs(gps - lidar) >= DISTANCE_THRESHOLD):
                out_tgt_relative_bearing_list.append(tgt_rel_bearing)
                out_own_relative_bearing_list.append(own_rel_bearing)

                in_tgt_relative_bearing_list.append(None)
                in_own_relative_bearing_list.append(None)

                ego_hdg_delta = None if old_ego_hdg == -math.inf else ego_hdg - old_ego_hdg 
                ego_hdg_delta = (ego_hdg_delta / abs(ego_hdg_delta)) * (abs(ego_hdg_delta) - 360) if ego_hdg_delta is not None and abs(ego_hdg_delta) > 100 else ego_hdg_delta
                out_ego_hdg_delta_list.append(ego_hdg_delta)

                tgt_hdg_delta = None if old_tgt_hdg == -math.inf else tgt_hdg - old_tgt_hdg 
                tgt_hdg_delta = (tgt_hdg_delta / abs(tgt_hdg_delta)) * (abs(tgt_hdg_delta) - 360) if tgt_hdg_delta is not None and abs(tgt_hdg_delta) > 100 else tgt_hdg_delta
                out_tgt_hdg_delta_list.append(tgt_hdg_delta)

                in_ego_hdg_delta_list.append(None)
                in_tgt_hdg_delta_list.append(None)

                out_distance_list.append(gps - lidar)
                in_distance_list.append(None)

            else:
                out_tgt_relative_bearing_list.append(None)
                out_own_relative_bearing_list.append(None)

                in_tgt_relative_bearing_list.append(tgt_rel_bearing)
                in_own_relative_bearing_list.append(own_rel_bearing)

                out_ego_hdg_delta_list.append(None)
                out_tgt_hdg_delta_list.append(None)

                ego_hdg_delta = None if old_ego_hdg == -math.inf else ego_hdg - old_ego_hdg 
                ego_hdg_delta = (ego_hdg_delta / abs(ego_hdg_delta)) * (abs(ego_hdg_delta) - 360) if ego_hdg_delta is not None and abs(ego_hdg_delta) > 100 else ego_hdg_delta
                in_ego_hdg_delta_list.append(ego_hdg_delta)

                tgt_hdg_delta = None if old_tgt_hdg == -math.inf else tgt_hdg - old_tgt_hdg 
                tgt_hdg_delta = (tgt_hdg_delta / abs(tgt_hdg_delta)) * (abs(tgt_hdg_delta) - 360) if tgt_hdg_delta is not None and abs(tgt_hdg_delta) > 100 else tgt_hdg_delta
                in_tgt_hdg_delta_list.append(tgt_hdg_delta)

                in_distance_list.append(gps - lidar)
                out_distance_list.append(None)

            old_ego_hdg, old_tgt_hdg = ego_hdg, tgt_hdg

        else:
            filtered_idx.append(idx[i])  # Add valid idx
            distance_difference.append(None)

            out_tgt_relative_bearing_list.append(None)
            out_own_relative_bearing_list.append(None)
            out_ego_hdg_delta_list.append(None)
            out_tgt_hdg_delta_list.append(None)

            in_tgt_relative_bearing_list.append(None)
            in_own_relative_bearing_list.append(None)
            in_ego_hdg_delta_list.append(None)
            in_tgt_hdg_delta_list.append(None)
            in_distance_list.append(None)
            out_distance_list.append(None)

    # Target Rel Bearing when Ego Heading is Zero 
    mean_dd = round(statistics.mean(valid_distance_diff), 3)
    std_dd = round(statistics.stdev(valid_distance_diff), 3)

    valid_tgt_relative_bearing = [bearing for bearing in in_tgt_relative_bearing_list+out_tgt_relative_bearing_list if bearing is not None]
    mean_target_rel_bearing = round(statistics.mean(valid_tgt_relative_bearing), 3)
    std_target_rel_bearing = round(statistics.stdev(valid_tgt_relative_bearing), 3)
    if (std_target_rel_bearing > 100):
        in_tgt_relative_bearing_list = [bearing - 360 if bearing is not None and bearing >= 180 else bearing for bearing in in_tgt_relative_bearing_list]
        out_tgt_relative_bearing_list = [bearing - 360 if bearing is not None and bearing >= 180 else bearing for bearing in out_tgt_relative_bearing_list]
        valid_tgt_relative_bearing = [bearing for bearing in in_tgt_relative_bearing_list+out_tgt_relative_bearing_list if bearing is not None]
        mean_target_rel_bearing = round(statistics.mean(valid_tgt_relative_bearing), 3)
        std_target_rel_bearing = round(statistics.stdev(valid_tgt_relative_bearing), 3)

    # Own Relative Bearing
    valid_own_relative_bearing = [bearing for bearing in in_own_relative_bearing_list+out_own_relative_bearing_list if bearing is not None]
    mean_ego_rel_bearing = round(statistics.mean(valid_own_relative_bearing), 3)
    std_ego_rel_bearing = round(statistics.stdev(valid_own_relative_bearing), 3)
    if (std_ego_rel_bearing > 100):
        in_own_relative_bearing_list = [bearing - 360 if bearing is not None and bearing >= 180 else bearing for bearing in in_own_relative_bearing_list]
        out_own_relative_bearing_list = [bearing - 360 if bearing is not None and bearing >= 180 else bearing for bearing in out_own_relative_bearing_list]
        valid_own_relative_bearing = [bearing for bearing in in_own_relative_bearing_list+out_own_relative_bearing_list if bearing is not None]
        mean_ego_rel_bearing = round(statistics.mean(valid_own_relative_bearing), 3)
        std_ego_rel_bearing = round(statistics.stdev(valid_own_relative_bearing), 3)

    # Ego Heading Delta 
    valid_own_hdg_delta = [hdg_delta for hdg_delta in in_ego_hdg_delta_list+out_ego_hdg_delta_list if hdg_delta is not None]
    mean_ego_hdg_delta = round(statistics.mean(valid_own_hdg_delta), 3)
    std_ego_hdg_delta = round(statistics.stdev(valid_own_hdg_delta), 3)
    if (std_ego_hdg_delta > 100):
        in_ego_hdg_delta_list = [hdg_delta - 360 if hdg_delta is not None and hdg_delta >= 180 else hdg_delta for hdg_delta in in_ego_hdg_delta_list]
        out_ego_hdg_delta_list = [hdg_delta - 360 if hdg_delta is not None and hdg_delta >= 180 else hdg_delta for hdg_delta in out_ego_hdg_delta_list]
        valid_own_hdg_delta = [hdg_delta for hdg_delta in in_ego_hdg_delta_list+out_ego_hdg_delta_list if hdg_delta is not None]
        mean_ego_hdg_delta = round(statistics.mean(valid_own_hdg_delta), 3)
        std_ego_hdg_delta = round(statistics.stdev(valid_own_hdg_delta), 3)

    # Target Heading 
    valid_tgt_hdg_delta = [hdg_delta for hdg_delta in in_tgt_hdg_delta_list+out_tgt_hdg_delta_list if hdg_delta is not None]
    mean_tgt_hdg_delta = round(statistics.mean(valid_tgt_hdg_delta), 3)
    std_tgt_hdg_delta = round(statistics.stdev(valid_tgt_hdg_delta), 3)
    if (std_ego_rel_bearing > 100):
        in_tgt_hdg_delta_list = [hdg_delta - 360 if hdg_delta is not None and hdg_delta >= 180 else hdg_delta for hdg_delta in in_tgt_hdg_delta_list]
        out_tgt_hdg_delta_list = [hdg_delta - 360 if hdg_delta is not None and hdg_delta >= 180 else hdg_delta for hdg_delta in out_tgt_hdg_delta_list]
        valid_tgt_hdg_delta = [hdg_delta for hdg_delta in in_tgt_hdg_delta_list+out_tgt_hdg_delta_list if hdg_delta is not None]
        mean_tgt_hdg_delta = round(statistics.mean(valid_tgt_hdg_delta), 3)
        std_tgt_hdg_delta = round(statistics.stdev(valid_tgt_hdg_delta), 3)

    # Create the graph
    fig, axs = plt.subplots(6, 1, figsize=(36, 20))  # Six stacked graphs (6x1)

    # First graph: Distance Comparison
    axs[0].plot(idx, gps_distance, color="blue", marker="o", label="GPS Distance")
    axs[0].plot(idx, lidar_distance, color="red", marker="s", label="LiDAR Distance")
    axs[0].set_title("Distance Comparison", fontsize=16)
    axs[0].set_xlabel("Index", fontsize=14)
    axs[0].set_ylabel("Distance", fontsize=14)
    axs[0].legend(loc="upper right")
    axs[0].grid(True)

    # Second graph: Distance Difference
    axs[1].plot(
        filtered_idx,
        in_distance_list,
        color="orange",
        marker="^",
        label="GPS - LiDAR Difference (Diff < {DISTANCE_THRESHOLD})",
    )
    axs[1].plot(
        filtered_idx,
        out_distance_list,
        color="red",
        marker="^",
        label="GPS - LiDAR Difference (Diff >= {DISTANCE_THRESHOLD})",
    )
    axs[1].set_title(f"GPS - LiDAR : mean({mean_dd}), std({std_dd})", fontsize=16)
    axs[1].set_xlabel("Index", fontsize=14)
    axs[1].set_ylabel("Difference", fontsize=14)

    x_min, x_max = min(idx), max(idx)  # Get the minimum and maximum of idx

    for i in range(len(axs)):
        axs[i].set_xlim(x_min, x_max)  # Set x-axis range for the first graph

    y_min, y_max = -10, 10 
    axs[1].set_ylim(y_min, y_max) 

    axs[1].axhline(
        y=0, color="red", linestyle="--", linewidth=1
    )  # Add a red dashed line
    axs[1].axhline(
        y=4, color="red", linestyle="--", linewidth=1
    )  # Add a red dashed line
    axs[1].legend(loc="upper right")
    axs[1].grid(True)

    # Third graph: Distance Difference
    axs[2].plot(
        filtered_idx,
        out_tgt_relative_bearing_list,
        color="red",
        marker="^",
        label=f"Target Relative Bearing (Diff >= {DISTANCE_THRESHOLD})",
    )
    axs[2].plot(
        filtered_idx,
        in_tgt_relative_bearing_list,
        color="blue",
        marker="^",
        label=f"Target Relative Bearing  (Diff < {DISTANCE_THRESHOLD})",
    )
    axs[2].set_title(f"Target Relative Bearing (set Ego Heading as Zero) : mean({mean_target_rel_bearing}), std({std_target_rel_bearing})", fontsize=16)
    axs[2].set_xlabel("Index", fontsize=14)
    axs[2].set_ylabel("Degree", fontsize=14)

    axs[2].axhline(
        y=0, color="red", linestyle="--", linewidth=1
    )  # Add a red dashed line
    axs[2].legend(loc="upper right")
    axs[2].grid(True)

    # Sixth graph: Own Relative Difference
    axs[3].plot(
        filtered_idx,
        out_own_relative_bearing_list,
        color="red",
        marker="*",
        label=f"Own Relative Bearing (Diff >= {DISTANCE_THRESHOLD})",
    )
    axs[3].plot(
        filtered_idx,
        in_own_relative_bearing_list,
        color="limegreen",
        marker="*",
        label=f"Own Relative Bearing  (Diff < {DISTANCE_THRESHOLD})",
    )
    axs[3].set_title(f"Own Relative Bearing : mean({mean_ego_rel_bearing}), std({std_ego_rel_bearing})", fontsize=16)
    axs[3].set_xlabel("Index", fontsize=14)
    axs[3].set_ylabel("Degree", fontsize=14)

    axs[3].axhline(
        y=0, color="red", linestyle="--", linewidth=1
    )  # Add a red dashed line
    axs[3].legend(loc="upper right")
    axs[3].grid(True)


    # Forth graph: Distance Difference
    axs[4].plot(
        filtered_idx,
        out_ego_hdg_delta_list,
        color="red",
        marker="o",
        label=f"Ego Heading Delta (Diff >= {DISTANCE_THRESHOLD})",
    )
    axs[4].plot(
        filtered_idx,
        in_ego_hdg_delta_list,
        color="orange",
        marker="o",
        label=f"Ego Heading (Diff < {DISTANCE_THRESHOLD})",
    )
    axs[4].set_title(f"Ego Heading Delta : mean({mean_ego_hdg_delta}), std({std_ego_hdg_delta})", fontsize=16)
    axs[4].set_xlabel("Index", fontsize=14)
    axs[4].set_ylabel("Degree", fontsize=14)

    axs[4].axhline(
        y=0, color="red", linestyle="--", linewidth=1
    )  # Add a red dashed line
    axs[4].legend(loc="upper right")
    axs[4].grid(True)


    # Fifth graph: Distance Difference
    axs[5].plot(
        filtered_idx,
        out_tgt_hdg_delta_list,
        color="red",
        marker="s",
        label=f"Target Heading (Diff >= {DISTANCE_THRESHOLD})",
    )
    axs[5].plot(
        filtered_idx,
        in_tgt_hdg_delta_list,
        color="green",
        marker="s",
        label=f"Target Heading  (Diff < {DISTANCE_THRESHOLD})",
    )
    axs[5].set_title(f"Target Heading Delta : mean({mean_tgt_hdg_delta}), std({std_tgt_hdg_delta})", fontsize=16)
    axs[5].set_xlabel("Index", fontsize=14)
    axs[5].set_ylabel("Degree", fontsize=14)

    axs[5].axhline(
        y=0, color="red", linestyle="--", linewidth=1
    )  # Add a red dashed line
    axs[5].legend(loc="upper right")
    axs[5].grid(True)


    # Adjust layout and display
    plt.tight_layout()
    plt.savefig(f"{result_path}/gps_lidar_difference.png")  # Save the graph
    plt.close(fig)  # Close the graph to release resources


def humanDate2epochTime(datetime_str):
    """
    Convert a 'YYYY/MM/DD HH:MM:SS.mmm' formatted datetime string to an epoch timestamp in milliseconds.
    """
    dt = datetime.strptime(datetime_str, "%Y/%m/%d %H:%M:%S.%f")
    kst = timezone(timedelta(hours=0))  # UTC+9
    dt_kst = dt.replace(tzinfo=kst)

    epoch_time = int(dt_kst.timestamp() * 1000)
    return epoch_time


def timestamp_ms_to_datetime_string(timestamp_ms):
    """
    Convert an epoch timestamp in milliseconds to a 'YYYY/MM/DD HH:MM:SS.mmm' formatted datetime string.
    """
    timestamp_s = timestamp_ms / 1000.0
    dt = datetime.fromtimestamp(timestamp_s)
    return dt.strftime("%Y/%m/%d %H:%M:%S.") + f"{int((timestamp_ms % 1000)):03d}"


def calculate_bearing(lat1: float, lon1: float, lat2: float, lon2: float):
    """
    Calculate the bearing between two points.
    """
    lat1 = math.radians(lat1)
    lon1 = math.radians(lon1)
    lat2 = math.radians(lat2)
    lon2 = math.radians(lon2)

    delta_lon = lon2 - lon1

    x = math.sin(delta_lon) * math.cos(lat2)
    y = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(
        delta_lon
    )

    initial_bearing = math.atan2(x, y)
    initial_bearing = math.degrees(initial_bearing)
    compass_bearing = (initial_bearing + 360) % 360

    return compass_bearing

def calculate_relative_bearing(lat1, lon1, lat2, lon2, heading):
    """
    Calculate the relative bearing from the ship's heading.
    """
    true_bearing = calculate_bearing(lat1, lon1, lat2, lon2)
    relative_bearing = (true_bearing - heading + 360) % 360  # 0~360° 범위 조정
    return relative_bearing

def create_directory(directory_path):
    # Create the directory if it doesn't exist
    if not os.path.exists(directory_path):
        os.makedirs(directory_path)
        print(f"Directory '{directory_path}' created.")
    else:
        print(f"Directory '{directory_path}' already exists.")


def caclulate_interpolated_radius(degree, radius_path, ego_gps_data):
    """
    Calculate the interpolated radius based on the given degree.
    """
    degree = degree % 360
    degree_floor = math.floor(degree / 10) * 10

    diff = degree - degree_floor
    df = pd.read_csv(radius_path)
    min_index = df[(df["angle"] <= degree) & (df["angle"] + 10 > degree)].index
    radius_min = df["radius"].iloc[min_index]
    radius_max = df["radius"].iloc[min_index + 1]
    radius_interpolated = (
        (radius_min.item()) * (10 - diff) + (radius_max.item()) * (diff)
    ) / 10
    return radius_interpolated / 297 * 8.89


# Convert Human Date to Epoch Time
def human2epoch(humanDate):
    return int(datetime.strptime(humanDate, "%Y/%m/%d %H:%M:%S.%f").timestamp() * 1000)


def create_parser():
    """Create and return an argument parser."""
    parser = argparse.ArgumentParser(
        description="A script that processes command-line arguments."
    )
    # Add arguments to the parser here
    parser.add_argument(
        "--root_path",
        type=str,
        default="../data/QUANTATIVE_TEST",
        help="Root path of data",
        required=False,
    )
    parser.add_argument(
        "--case_name", type=str, default="000", help="Case name", required=False
    )
    parser.add_argument(
        "--ego_boat", type=str, default="as280", help="Type of ego boat", required=False
    )
    parser.add_argument(
        "--tgt_boat",
        type=str,
        default="as280",
        help="Type of target boat",
        required=False,
    )
    parser.add_argument(
        "--track_id",
        type=str,
        default="2",
        help="Comma-separated list of track IDs",
        required=True,
    )
    return parser


def get_target_cp_from_csv(lidar_cp_path: str, gps_distance: float, track_ids:list):
    df = pd.read_csv(lidar_cp_path)
    if df.empty:
        return None, None, None

    try:
        if len(track_ids) == 0:
            print("Warning: Please input track IDs.")
            raise TrackIdError("Track IDs are required.")

        df_boat = df[df["trackid"].isin(track_ids)].copy()

        df_boat["distance"] = df_boat.apply(
            lambda row: np.sqrt(row["position_x"] ** 2 + row["position_y"] ** 2), axis=1
        )
        df_boat = df_boat.dropna(subset=["distance"])
        df_boat["distance_diff"] = (df_boat["distance"] - gps_distance).abs()
        min_diff_index = df_boat["distance_diff"].idxmin()
        closest_item = df_boat.loc[min_diff_index, "distance"].item()
        if abs(closest_item - gps_distance) > 20:
            return None, None, None
        return closest_item, df_boat.loc[min_diff_index, "trackid"].item(), df_boat.loc[min_diff_index, "objtype"].item()
    except TrackIdError as e:
        print(e)
        sys.exit("Program terminated due to missing track IDs.")
    except:
        return None, None, None


def get_closest_lidar_csv(sync_lidar_ts, lidar_path_list):
    if not lidar_path_list:
        raise ValueError("lidar_path_list is empty")

    min_diff_ts = math.inf
    min_lidar_path = ""

    for lidar_path in lidar_path_list:
        lidar_ts = int(lidar_path.split("_")[0])
        diff = abs(lidar_ts - sync_lidar_ts)
        if diff < min_diff_ts:
            min_diff_ts = diff
            min_lidar_path = lidar_path
    return min_lidar_path


def main():
    # 0. Argument parsing
    parser = create_parser()
    args = parser.parse_args()

    root_path = args.root_path
    ego_case_name = args.case_name
    tgt_case_name = args.case_name
    track_ids = list(map(int, args.track_id.split(",")))  # Convert comma-separated track IDs into a list

    ego = "TARGET"
    target = "OWN"

    result_path = (
        f"{root_path}/result/{ego}_{ego_case_name}"  # Save notebook in the ego folder
    )
    create_directory(result_path)
    
    radius_csv_map = {
        "as280": "../config/as280_vessel_radius_by_angle.csv",
        "sdx290": "../config/sdx290_vessel_radius_by_angle.csv",
    }

    ego_sync_path = f"{root_path}/testset/{ego}_{ego_case_name}/oru/sync.csv"
    ego_gps_path = f"{root_path}/testset/{ego}_{ego_case_name}/oru/position.csv"
    tgt_sync_path = f"{root_path}/testset/{target}_{tgt_case_name}/oru/sync.csv"
    tgt_gps_path = f"{root_path}/testset/{target}_{tgt_case_name}/oru/position.csv"

    ego_sync_df = pd.read_csv(ego_sync_path, header=None)
    ego_gps_df = pd.read_csv(ego_gps_path, header=None)

    tgt_sync_df = pd.read_csv(tgt_sync_path, header=None)
    tgt_gps_df = pd.read_csv(tgt_gps_path, header=None)

    ego_boat_type = args.ego_boat
    tgt_boat_type = args.tgt_boat

    # 1. Read files one by one from the ego's sync.csv
    ego_sync_gps_column = ego_sync_df.iloc[:, -1]
    tgt_sync_gps_column = tgt_sync_df.iloc[:, -1]

    ego_lidar_path = f"{root_path}/targets/{ego}_{ego_case_name}"
    lidar_path_list = sorted(
        [f for f in os.listdir(ego_lidar_path) if f.endswith(".csv")],
        key=lambda x: int(x.split("_")[0]),
    )

    idx_list, gps_distance_list, lidar_distance_list, tgt_relative_bearing_list = [], [], [], []
    ego_hdg_list, tgt_hdg_list, own_relative_bearing_list = [], [], []

    output_file = f"{result_path}/gps_lidar_output.csv"
    with open(output_file, mode="w", newline="") as file:
        writer = csv.writer(file)

        # Write headers
        writer.writerow(
            [
                "idx",
                "ego_gps_timestamp",
                "ego_lat",
                "ego_lon",
                "tgt_gps_timestamp",
                "tgt_lat",
                "tgt_lon",
                "ego_hdg",
                "tgt_hdg",
                "bearing",
                "own_relative_bearing",
                "tgt_relative_bearing",
                "gps_distance",
                "lidar_distance",                
                "track_id",
                "class_id",
            ]
        )

        for idx, ego_sync_gps_ts in enumerate(ego_sync_gps_column):
            ego_sync_gps_ts_epoch = humanDate2epochTime(ego_sync_gps_ts)
            tgt_sync_gps_epoch = tgt_sync_gps_column.apply(humanDate2epochTime)
            ego_tgt_time_diff = (tgt_sync_gps_epoch - ego_sync_gps_ts_epoch).abs()
            idx_min_ego_tgt_time_diff = ego_tgt_time_diff.idxmin()

            if ego_tgt_time_diff[idx_min_ego_tgt_time_diff] > THRESHOLD_EGO_TGT_GPS:
                print(
                    f"Time diff between ego and tgt is {ego_tgt_time_diff[idx_min_ego_tgt_time_diff]}"
                )
                continue

            # 1. Get ego position data
            ego_sync_gps_ts_diff = (
                ego_gps_df.iloc[:, 0] - ego_sync_df.iloc[idx, 0]
            ).abs()
            idx_min_ego_gps_time_diff = ego_sync_gps_ts_diff.idxmin()

            if (
                ego_sync_gps_ts_diff.iloc[idx_min_ego_gps_time_diff]
                > THRESHOLD_EGO_TGT_GPS
            ):
                print(
                    f"Time diff between ego_sync and ego_gps is {ego_sync_gps_ts_diff.iloc[idx_min_ego_gps_time_diff]}"
                )
                continue

            ego_gps_data = ego_gps_df.iloc[idx_min_ego_gps_time_diff]

            # 2. Get target position data
            tgt_sync_gps_ts_diff = (
                tgt_gps_df.iloc[:, 0] - tgt_sync_df.iloc[idx_min_ego_tgt_time_diff, 0]
            ).abs()
            idx_min_tgt_gps_time_diff = tgt_sync_gps_ts_diff.idxmin()
            if (
                tgt_sync_gps_ts_diff.iloc[idx_min_tgt_gps_time_diff]
                > THRESHOLD_EGO_TGT_GPS
            ):
                print(
                    f"Time diff between tgt_sync and tgt_gps is {tgt_sync_gps_ts_diff.iloc[idx_min_tgt_gps_time_diff]}"
                )
                continue

            tgt_gps_data = tgt_gps_df.iloc[idx_min_tgt_gps_time_diff]

            # 3. Get lidar data
            lidar_csv = (
                ego_lidar_path
                + "/"
                + get_closest_lidar_csv(ego_gps_data[0], lidar_path_list)
            )  # 0th data = lidar timestamp

            # 4. GPS data handling
            _, ego_lat, ego_lon, _, ego_hdg = list(ego_gps_data)  # lat, lon, cog, hdg
            _, tgt_lat, tgt_lon, _, tgt_hdg = list(tgt_gps_data)

            # 1) Calculate bearing
            bearing = calculate_bearing(ego_lat, ego_lon, tgt_lat, tgt_lon)

            # 2) Adjust target to have ego heading as 0, then measure relative bearing
            # Relative bearing: the angle measured clockwise from the target's perspective
            own_relative_bearing = calculate_relative_bearing(ego_lat, ego_lon, tgt_lat, tgt_lon, ego_hdg)
            tgt_relative_bearing = (180 + own_relative_bearing + ego_hdg - tgt_hdg) % 360

            # 3) Read table based on the bearing angle to get values
            ego_boat_radius_path = radius_csv_map[ego_boat_type]
            tgt_boat_radius_path = radius_csv_map[tgt_boat_type]

            try:
                ego_radius = caclulate_interpolated_radius(
                    own_relative_bearing, ego_boat_radius_path, ego_gps_data
                )
                tgt_radius = caclulate_interpolated_radius(
                    tgt_relative_bearing, tgt_boat_radius_path, tgt_gps_data
                )
            except:
                continue

            # 4) Calculate the distance between the target's GPS and the ego's GPS until the surface of the target
            gps_distance = (
                distance.distance((ego_lat, ego_lon), (tgt_lat, tgt_lon)).m
                - tgt_radius
                - ego_radius
            )

            # 5) Get data from lidar path
            lidar_distance, track_id, class_id = get_target_cp_from_csv(lidar_csv, gps_distance, track_ids)

            if lidar_distance == None:
                print(f"gps distance {gps_distance:<20} | lidar_distance {lidar_distance}")
            else:
                print(f"gps distance {gps_distance:<20} | lidar_distance {lidar_distance:<20}")

            idx_list.append(idx)
            gps_distance_list.append(gps_distance)
            lidar_distance_list.append(lidar_distance)
            tgt_relative_bearing_list.append(tgt_relative_bearing)
            ego_hdg_list.append(ego_hdg)
            tgt_hdg_list.append(tgt_hdg)
            own_relative_bearing_list.append(own_relative_bearing)

            writer.writerow(
                [
                    idx,
                    ego_gps_data.iloc[0],
                    ego_lat,
                    ego_lon,
                    tgt_gps_data.iloc[0],
                    tgt_lat,
                    tgt_lon,
                    ego_hdg,
                    tgt_hdg,
                    bearing,
                    own_relative_bearing,
                    tgt_relative_bearing,
                    gps_distance,
                    lidar_distance,
                    track_id,
                    class_id,
                ]
            )


        draw_graph(idx_list, gps_distance_list, lidar_distance_list, tgt_relative_bearing_list, ego_hdg_list, tgt_hdg_list, own_relative_bearing_list, result_path)
        print(f"Distance comparison graph (GPS vs LiDAR) saved at : '{result_path}/gps_lidar_difference.png'.")

if __name__ == "__main__":
    main()
