#!/usr/bin/env python3
"""使用棋盘格图片标定相机，并生成本工程使用的标定配置。"""

import argparse
from pathlib import Path
import sys

import cv2
import numpy as np


def collect_images(directory: Path) -> list[Path]:
    extensions = {".jpg", ".jpeg", ".png", ".bmp"}
    return sorted(
        path for path in directory.iterdir()
        if path.is_file() and path.suffix.lower() in extensions
    )


def build_object_points(
    columns: int,
    rows: int,
    board_width_m: float,
    board_height_m: float,
) -> tuple[np.ndarray, float, float]:
    # 9×6内角点对应10×7个完整方格。
    spacing_x = board_width_m / (columns + 1)
    spacing_y = board_height_m / (rows + 1)
    points = np.zeros((columns * rows, 3), np.float32)
    grid = np.mgrid[0:columns, 0:rows].T.reshape(-1, 2)
    points[:, 0] = grid[:, 0] * spacing_x
    points[:, 1] = grid[:, 1] * spacing_y
    return points, spacing_x, spacing_y


def write_config(
    output: Path,
    width: int,
    height: int,
    camera_matrix: np.ndarray,
    distortion: np.ndarray,
    armor_width_m: float,
    armor_height_m: float,
    max_reprojection_error_px: float,
) -> None:
    coefficients = np.zeros(5, dtype=np.float64)
    flattened = distortion.reshape(-1)
    coefficients[: min(5, flattened.size)] = flattened[:5]
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        "# 由scripts/calibrate_camera.py生成。\n"
        "enabled=true\n\n"
        f"image_width={width}\n"
        f"image_height={height}\n"
        f"fx={camera_matrix[0, 0]:.12g}\n"
        f"fy={camera_matrix[1, 1]:.12g}\n"
        f"cx={camera_matrix[0, 2]:.12g}\n"
        f"cy={camera_matrix[1, 2]:.12g}\n"
        f"k1={coefficients[0]:.12g}\n"
        f"k2={coefficients[1]:.12g}\n"
        f"p1={coefficients[2]:.12g}\n"
        f"p2={coefficients[3]:.12g}\n"
        f"k3={coefficients[4]:.12g}\n\n"
        f"armor_width_m={armor_width_m:.12g}\n"
        f"armor_height_m={armor_height_m:.12g}\n"
        f"max_reprojection_error_px={max_reprojection_error_px:.12g}\n",
        encoding="utf-8",
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("image_directory", type=Path, help="标定图片目录")
    parser.add_argument("--columns", type=int, default=9, help="横向内角点数")
    parser.add_argument("--rows", type=int, default=6, help="纵向内角点数")
    parser.add_argument(
        "--board-width-m", type=float, default=0.247,
        help="黑白格有效区域总宽度，单位米",
    )
    parser.add_argument(
        "--board-height-m", type=float, default=0.170,
        help="黑白格有效区域总高度，单位米",
    )
    parser.add_argument("--target-width", type=int, default=1920)
    parser.add_argument("--target-height", type=int, default=1080)
    parser.add_argument("--armor-width-m", type=float, default=0.135)
    parser.add_argument("--armor-height-m", type=float, default=0.056)
    parser.add_argument("--max-reprojection-error-px", type=float, default=3.0)
    parser.add_argument(
        "--output", type=Path,
        default=Path("calibration/camera_calibration.ini"),
    )
    arguments = parser.parse_args()

    if not arguments.image_directory.is_dir():
        print(f"图片目录不存在: {arguments.image_directory}", file=sys.stderr)
        return 2
    if arguments.columns < 2 or arguments.rows < 2:
        print("内角点行列数必须至少为2", file=sys.stderr)
        return 2
    if arguments.board_width_m <= 0 or arguments.board_height_m <= 0:
        print("棋盘有效区域尺寸必须为正数", file=sys.stderr)
        return 2

    image_paths = collect_images(arguments.image_directory)
    if not image_paths:
        print("目录中没有标定图片", file=sys.stderr)
        return 2

    object_template, spacing_x, spacing_y = build_object_points(
        arguments.columns,
        arguments.rows,
        arguments.board_width_m,
        arguments.board_height_m,
    )
    pattern_size = (arguments.columns, arguments.rows)
    object_points: list[np.ndarray] = []
    image_points: list[np.ndarray] = []
    accepted_paths: list[Path] = []
    image_size = None

    find_flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
    criteria = (
        cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER,
        50,
        1e-4,
    )

    for image_path in image_paths:
        image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
        if image is None:
            print(f"跳过无法读取的图片: {image_path.name}")
            continue
        current_size = (image.shape[1], image.shape[0])
        if image_size is None:
            image_size = current_size
        if current_size != image_size:
            print(
                f"跳过分辨率不一致的图片: {image_path.name} "
                f"{current_size[0]}x{current_size[1]}"
            )
            continue

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCorners(
            gray, pattern_size, flags=find_flags
        )
        if not found:
            print(f"未找到完整棋盘角点: {image_path.name}")
            continue
        refined = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        object_points.append(object_template.copy())
        image_points.append(refined)
        accepted_paths.append(image_path)

    if image_size is None or len(accepted_paths) < 15:
        print(
            f"有效图片只有{len(accepted_paths)}张，至少需要15张，建议30至50张",
            file=sys.stderr,
        )
        return 3

    rms, camera_matrix, distortion, rotation_vectors, translation_vectors = (
        cv2.calibrateCamera(
            object_points,
            image_points,
            image_size,
            None,
            None,
        )
    )

    view_errors = []
    for object_set, image_set, rotation, translation in zip(
        object_points,
        image_points,
        rotation_vectors,
        translation_vectors,
    ):
        projected, _ = cv2.projectPoints(
            object_set, rotation, translation, camera_matrix, distortion
        )
        residual = image_set.reshape(-1, 2) - projected.reshape(-1, 2)
        error = np.sqrt(np.mean(np.sum(residual * residual, axis=1)))
        view_errors.append(float(error))

    scale_x = arguments.target_width / image_size[0]
    scale_y = arguments.target_height / image_size[1]
    target_matrix = camera_matrix.copy()
    target_matrix[0, 0] *= scale_x
    target_matrix[0, 2] *= scale_x
    target_matrix[1, 1] *= scale_y
    target_matrix[1, 2] *= scale_y

    write_config(
        arguments.output,
        arguments.target_width,
        arguments.target_height,
        target_matrix,
        distortion,
        arguments.armor_width_m,
        arguments.armor_height_m,
        arguments.max_reprojection_error_px,
    )

    print(f"读取图片: {len(image_paths)}")
    print(f"有效图片: {len(accepted_paths)}")
    print(f"输入分辨率: {image_size[0]}x{image_size[1]}")
    print(f"输出内参分辨率: {arguments.target_width}x{arguments.target_height}")
    print(f"横向格距: {spacing_x * 1000:.4f} mm")
    print(f"纵向格距: {spacing_y * 1000:.4f} mm")
    print(f"OpenCV RMS: {rms:.6f} px")
    print(f"平均单图误差: {np.mean(view_errors):.6f} px")
    print(f"最大单图误差: {np.max(view_errors):.6f} px")
    print("误差最大的图片:")
    worst_views = sorted(
        zip(view_errors, accepted_paths), reverse=True, key=lambda item: item[0]
    )[:5]
    for error, image_path in worst_views:
        print(f"  {error:.6f} px  {image_path.name}")
    print(f"配置文件: {arguments.output.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
