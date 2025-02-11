import cv2
import os
import argparse


def create_video_from_images(image_list_path, output_video_path, frame_rate=30):
    """
    Creates a video from images in a specified folder.

    Args:
        image_list_path (str): Path to the folder containing images.
        output_video_path (str): Path to save the generated video.
        frame_rate (int): Frame rate of the output video.
    """
    image_files = [f for f in os.listdir(image_list_path) if f.endswith(".jpg")]
    image_files.sort(key=lambda x: int(os.path.splitext(x)[0]))  # Sort numerically

    if not image_files:
        print("No images found in the folder.")
        return

    first_image = cv2.imread(os.path.join(image_list_path, image_files[0]))
    height, width, _ = first_image.shape

    fourcc = cv2.VideoWriter_fourcc(*"mp4v")  # 'mp4v' codec for .mp4 output
    video_writer = cv2.VideoWriter(
        output_video_path, fourcc, frame_rate, (width, height)
    )

    for image_file in image_files:
        image_path = os.path.join(image_list_path, image_file)
        image = cv2.imread(image_path)
        video_writer.write(image)

    video_writer.release()
    print(f"Detection results video saved successfully at '{output_video_path}'.")


def create_parser():
    """
    Creates and returns an argument parser for command-line inputs.
    """
    parser = argparse.ArgumentParser(
        description="A script that processes command-line arguments."
    )
    parser.add_argument(
        "--root_path",
        type=str,
        default="../data/QUANTATIVE_TEST",
        help="Root path of data",
        required=True
    )
    parser.add_argument("--case_name", type=str, default="000", help="Case name", required=True)
    return parser


if __name__ == "__main__":
    parser = create_parser()
    args = parser.parse_args()

    root_path = args.root_path
    case_name = args.case_name
    own_tgt = "TARGET"  # Hardcoded target name

    image_list_path = f"{root_path}/testset/{own_tgt}_{case_name}/camera"
    video_result_path = f"{root_path}/result/{own_tgt}_{case_name}/video.mp4"
    frame_rate = 30  # Frame rate of the output video

    create_video_from_images(image_list_path, video_result_path, frame_rate)
