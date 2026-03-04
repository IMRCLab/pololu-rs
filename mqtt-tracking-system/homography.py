#!/usr/bin/env python3
"""
Interactive homography calibration:
- Shows a live camera feed (press 's' to freeze a frame)
- Click 4 arena corners in this order:
    1) bottom-left, 2) bottom-right, 3) top-right, 4) top-left
- Computes a 3x3 homography that maps image pixels -> world coordinates (meters)
- Writes the resulting matrix into homography.npy
"""

from pathlib import Path

import cv2
import numpy as np
import yaml


def _resolve_config_path(cli_arg: str | None = None) -> Path:
    if cli_arg:
        return Path(cli_arg)
    cwd_cfg = Path("config.yaml")
    if cwd_cfg.exists():
        return cwd_cfg
    return Path(__file__).with_name("config.yaml")


def load_config(path: Path) -> dict:
    if not path.exists():
        raise FileNotFoundError(f"Config file not found: {path}")
    with path.open("r", encoding="utf-8") as f:
        return yaml.safe_load(f) or {}


def _fourcc_from_str(s: str) -> int:
    s = (s or "MJPG").strip()
    if len(s) != 4:
        s = "MJPG"
    return cv2.VideoWriter_fourcc(*s)


def main() -> None:
    import argparse

    parser = argparse.ArgumentParser(description="Compute and save pixel->world homography to config.yaml")
    parser.add_argument("--config", default=None, help="Path to config.yaml (default: ./config.yaml or next to script)")
    args = parser.parse_args()

    config_path = _resolve_config_path(args.config)
    cfg = load_config(config_path)

    board_width_m = float(cfg["arena"]["width_m"])
    board_height_m = float(cfg["arena"]["height_m"])

    # World points (meters)
    world_points = np.array(
        [
            [0.0, 0.0],
            [board_width_m, 0.0],
            [board_width_m, board_height_m],
            [0.0, board_height_m],
        ],
        dtype=np.float32,
    )

    cam_cfg = cfg.get("camera", {})
    cam_index = int(cam_cfg.get("index", 0))
    frame_w = int(cam_cfg.get("frame_width", 3840))
    frame_h = int(cam_cfg.get("frame_height", 2160))
    autofocus = bool(cam_cfg.get("autofocus", False))
    focus = int(cam_cfg.get("focus", 0))
    fourcc = _fourcc_from_str(str(cam_cfg.get("fourcc", "MJPG")))

    clicked_points: list[list[int]] = []

    def click_event(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN and len(clicked_points) < 4:
            clicked_points.append([int(x), int(y)])
            print(f"Point {len(clicked_points)}: ({x}, {y})")

    cap = cv2.VideoCapture(cam_index)
    cap.set(cv2.CAP_PROP_FOURCC, fourcc)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, frame_w)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, frame_h)
    cap.set(cv2.CAP_PROP_AUTOFOCUS, 1 if autofocus else 0)
    cap.set(cv2.CAP_PROP_FOCUS, focus)

    print("Press 's' to capture a frame for selecting points. Press 'q' to quit.")
    frame = None

    while True:
        ret, frame = cap.read()
        if not ret:
            frame = None
            break
        cv2.namedWindow("Live Camera Feed - Press 's' to select", cv2.WINDOW_NORMAL)
        cv2.imshow("Live Camera Feed - Press 's' to select", frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("s"):
            break
        if key == ord("q"):
            frame = None
            break

    cv2.destroyAllWindows()
    cap.release()

    if frame is None:
        print("Failed to capture frame from camera.")
        return

    clone = frame.copy()
    win = "Click 4 corners (BL, BR, TR, TL), then press s"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    cv2.setMouseCallback(win, click_event)

    while True:
        cv2.imshow(win, clone)
        key = cv2.waitKey(50) & 0xFF
        if key == ord("q"):
            clicked_points.clear()
            break
        if len(clicked_points) == 4 and key == ord("s"):
            break

    cv2.destroyAllWindows()

    if len(clicked_points) != 4:
        print("You need to click exactly 4 points.")
        return

    print("Calculating homography...")
    image_points = np.array(clicked_points, dtype=np.float32)
    H, _ = cv2.findHomography(image_points, world_points)
    if H is None:
        print("Homography estimation failed.")
        return

    # Print Homography matrix
    print("Homography matrix (pixel -> world [m]):")
    print("H = np.array([")
    for row in H:
        print("    [" + ", ".join(f"{float(v):.8f}" for v in row) + "],")
    print("], dtype=np.float32)")

    # Save to homography.npy
    np.save("homography.npy", H)
    print(f"Updated homography.npy: {config_path.resolve()}")

    # Example usage
    def pixel_to_world(pt):
        px = np.array([[pt[0], pt[1], 1.0]], dtype=np.float32).T
        world_pt = H @ px
        world_pt /= world_pt[2]
        return world_pt[0:2].flatten()

    test_px = (400, 300)
    world_coord = pixel_to_world(test_px)
    print(f"Pixel {test_px} -> World coords [m]: {world_coord}")


if __name__ == "__main__":
    main()
