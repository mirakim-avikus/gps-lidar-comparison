import argparse
import os
import json
import cv2
import numpy as np
import pandas as pd
from make_video import create_video_from_images

# Class colors mapping for different object types
class_colors = {
    0: (0, 128, 0),  # Class 0: Dark Green (Subtle tone)
    1: (0, 0, 255),  # Class 1: Deep Orange (Bright color)
    2: (70, 130, 180),  # Class 2: Steel Blue (Subtle blue tone)
    3: (255, 215, 0),  # Class 3: Gold (Slightly softer yellow)
    4: (186, 85, 211),  # Class 4: Medium Orchid (Pinkish purple)
    5: (135, 206, 235),  # Class 5: Sky Blue (Light sky blue)
}

# Function to draw class legends (names and corresponding colors) on the image
def draw_class_legend(img):
    y_offset = 30  # Y-coordinate where the first class label is placed (positioning from top)
    font_scale = 0.7  # Text size for the class labels
    font_thickness = 2  # Text thickness
    font_color = (0, 0, 0)  # Text color (black)

    # Display each class name and its corresponding color on the image
    for class_id, color in class_colors.items():
        class_name = f"class{class_id}"  # Class name (e.g., class0, class1, ...)
        cv2.putText(
            img,
            class_name,
            (10, y_offset),  # Position for the class name
            cv2.FONT_HERSHEY_SIMPLEX,  # Font type
            font_scale,
            color,  # Text color
            font_thickness,
        )
        y_offset += 30  # Increase y_offset to avoid overlap between class names


# Create and return the argument parser for command-line inputs
def create_parser():
    parser = argparse.ArgumentParser(description="A script that processes command-line arguments.")
    parser.add_argument(
        "--root_path", type=str, default="../data/QUANTATIVE_TEST", help="Root path of data", required=False
    )
    parser.add_argument(
        "--case_name", type=str, default="000", help="Case name", required=False
    )
    parser.add_argument(
        "--ego_boat", type=str, default="as280", help="Type of ego boat", required=False
    )
    parser.add_argument(
        "--tgt_boat", type=str, default="as280", help="Type of target boat", required=False
    )
    return parser


# Extract target timestamp from the data filename (based on first part before the underscore)
def get_target_ts(data):
    return int(data.split("_")[0])


# Extract camera timestamp from the filename (based on part before the dot)
def get_cam_ts(data):
    return int(data.split(".")[0])


# Function to check if annotations and target data match correctly
def check_anno_targets(json_path, csv_path):
    with open(json_path, "r") as f:
        data = json.load(f)

    # Read the CSV file containing target data
    df = pd.read_csv(csv_path)

    # Extract object types from the annotation JSON
    json_obj_types = [obj["obj_type"] for obj in data]

    # Count occurrences of each obj_type in the CSV data
    obj_type_counts = {}
    for obj_type in json_obj_types:
        count = df[df["objtype"] == obj_type].shape[0]
        obj_type_counts[obj_type] = count


# Function to draw bounding boxes on the image
def draw_bbox_on_img(img_path, anno_path, csv_path, img_result_path):
    global idx  # Global index variable, although it isn't being used here

    # Load annotation JSON data
    with open(anno_path, "r") as f:
        json_data = json.load(f)

    # Read the target CSV file
    df = pd.read_csv(csv_path)

    # Read the camera image
    img = cv2.imread(img_path)

    # Draw class legends on the image for better understanding
    draw_class_legend(img)

    # Iterate through each object in the annotation data
    for obj in json_data:
        obj_type = obj["obj_type"]

        # Skip objects with obj_type 6 (likely unwanted or invalid objects)
        if obj_type == 6:
            continue

        bbox = obj["bbox"]  # Bounding box in the format [left, top, width, height]
        img_left_x, img_top_y, img_width, img_height = bbox

        # Filter the CSV data by obj_type to match relevant target data
        matching_rows = df[df["objtype"] == obj_type]
        color = class_colors.get(obj_type, (255, 255, 255))  # Default to white if no match

        # Calculate Euclidean distance for each matched row (useful for positioning)
        for _, row in matching_rows.iterrows():
            position_x = row["position_x"]
            position_y = row["position_y"]

            # Extract image-related data for bounding box
            img_left_x = int(row["img_left_x"].item())
            img_top_y = int(row["img_top_y"].item())
            img_width = int(row["img_width"].item())
            img_height = int(row["img_height"].item())

            track_id = int(row["trackid"].item())  # Track ID for the object

            # Calculate the Euclidean distance (useful for object localization)
            distance = np.sqrt((position_x) ** 2 + (position_y) ** 2)

            # Draw the bounding box around the object
            cv2.rectangle(
                img,
                (img_left_x, img_top_y),
                (img_left_x + img_width, img_top_y + img_height),
                color,  # Color based on object type
                2,  # Thickness of the bounding box
            )

            # Add track ID text on top of the bounding box
            obj_type_track_id_text = f"[{track_id}]"
            obj_type_track_id_position = (
                (img_left_x, img_top_y - 10)
                if img_top_y - 10 > 0
                else (img_left_x, img_top_y + img_height + 10)
            )

            # Add track ID text to the image
            cv2.putText(
                img,
                obj_type_track_id_text,
                obj_type_track_id_position,
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                color,
                2,
            )

            # Add distance text below the bounding box
            distance_text = f"{distance:.2f}"
            distance_position = (img_left_x, img_top_y + img_height + 20)  # Position text below the bbox

            # Add distance text to the image
            cv2.putText(
                img,
                distance_text,
                distance_position,
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                color,
                2,
            )

    # Save the annotated image with bounding boxes and text
    imgname = os.path.basename(img_path)
    cv2.imwrite(f"{img_result_path}/{imgname}", img)


# Main function to coordinate the data processing
def main():
    # 0. Argument parsing: parsing command-line arguments
    parser = create_parser()
    args = parser.parse_args()

    root_path = args.root_path
    case_name = args.case_name
    own_tgt = "TARGET"  # Hardcoded target name

    # Define paths to the necessary data
    cam_path = f"{root_path}/testset/{own_tgt}_{case_name}/camera"
    anno_path = f"{root_path}/testset/{own_tgt}_{case_name}/fcam_out"
    target_path = f"{root_path}/targets/{own_tgt}_{case_name}"

    # Define path for saving result images
    img_result_path = f"{root_path}/result/{own_tgt}_{case_name}/images"
    os.makedirs(img_result_path, exist_ok=True)
    output_video_path = f"{root_path}/result/{own_tgt}_{case_name}/video.mp4"  # Output video file path

    # Get the list of camera and target files
    cam_file_list = os.listdir(cam_path)
    jpg_files = [file for file in cam_file_list if file.endswith(".jpg")]
    sorted_cam_files = sorted(jpg_files, key=lambda x: int(x.split(".")[0]))

    target_file_list = os.listdir(target_path)
    sorted_target_files = sorted(target_file_list, key=lambda x: int(x.split("_")[0]))

    camera_idx = 0
    target_idx = 0

    # Synchronize camera and target files based on timestamps
    while camera_idx < len(sorted_cam_files):
        while target_idx + 1 < len(sorted_target_files) and abs(
            get_cam_ts(sorted_cam_files[camera_idx])
            - get_target_ts(sorted_target_files[target_idx])
        ) > abs(
            get_cam_ts(sorted_cam_files[camera_idx])
            - get_target_ts(sorted_target_files[target_idx + 1])
        ):
            target_idx += 1

        # Construct the paths to the current camera, annotation, and target data
        cam_data_path = f"{cam_path}/{sorted_cam_files[camera_idx]}"
        anno_data_path = f"{anno_path}/{get_cam_ts(sorted_cam_files[camera_idx])}.json"
        tgt_data_path = f"{target_path}/{sorted_target_files[target_idx]}"

        # Skip processing if the annotation data is invalid or empty
        with open(anno_data_path, "r") as f:
            json_data = json.load(f)
            if not json_data:
                camera_idx += 1
                continue

        # Draw bounding boxes and save the annotated image
        draw_bbox_on_img(cam_data_path, anno_data_path, tgt_data_path, img_result_path)
        camera_idx += 1

    # Create a video from the annotated images
    create_video_from_images(img_result_path, output_video_path)


if __name__ == "__main__":
    main()
