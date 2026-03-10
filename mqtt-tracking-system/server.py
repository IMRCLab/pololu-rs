import paho.mqtt.client as mqtt
import json
import yaml
import numpy as np
import sys
import cv2
import math
import threading
import os
import time
import csv
from datetime import datetime


DEFAULT_CONFIG_PATH = "config.yaml"
DEFAULT_HOMOGRAPHY_PATH = "homography.npy"


def _get_required(cfg: dict, path: str):
    """Fetch a required config value using dotted-path notation (e.g. 'mqtt.broker')."""
    cur = cfg
    for key in path.split("."):
        if not isinstance(cur, dict) or key not in cur:
            raise KeyError(path)
        cur = cur[key]
    return cur


def load_config(config_path: str) -> dict:
    if not os.path.exists(config_path):
        print(f"ERROR: Config file '{config_path}' not found.")
        sys.exit(1)

    with open(config_path, "r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f) or {}

    # Validate presence of the fields that exist in your config.yaml.
    required_paths = [
        "arena.width_m",
        "arena.height_m",
        "camera.index",
        "camera.fourcc",
        "camera.frame_width",
        "camera.frame_height",
        "camera.autofocus",
        "camera.focus",
        "mqtt.broker",
        "mqtt.port",
        "mqtt.topic",
        "visualization.enabled",
        "visualization.warp_scale",
    ]
    missing = []
    for p in required_paths:
        try:
            _get_required(cfg, p)
        except KeyError:
            missing.append(p)

    if missing:
        print("ERROR: config.yaml is missing required keys:")
        for p in missing:
            print(f"  - {p}")
        sys.exit(1)

    return cfg


def load_homography(homography_path: str) -> np.ndarray:
    if not os.path.exists(homography_path):
        print(
            f"ERROR: Homography file '{homography_path}' not found. "
            "Please run your homography creation script (the one that saves homography.npy) first."
        )
        sys.exit(1)

    try:
        H = np.load(homography_path)
    except Exception as e:
        print(
            f"ERROR: Failed to load homography from '{homography_path}': {e}. "
            "Please re-generate it using homography creation script."
        )
        sys.exit(1)

    H = np.asarray(H)
    if H.size == 0:
        print(
            f"ERROR: Homography matrix in '{homography_path}' is empty. "
            "Please run homography creation script to generate a valid matrix."
        )
        sys.exit(1)

    if H.shape != (3, 3):
        print(
            f"ERROR: Homography matrix in '{homography_path}' has shape {H.shape}, expected (3, 3). "
            "Please re-generate it using homography creation script."
        )
        sys.exit(1)

    if not np.isfinite(H).all() or not np.any(np.abs(H) > 1e-12):
        print(
            f"ERROR: Homography matrix in '{homography_path}' is invalid (NaN/Inf or all zeros). "
            "Please re-generate it using homography creation script."
        )
        sys.exit(1)

    return H.astype(np.float32)

class Camera:
    def __init__(self, camera_cfg: dict):
        index = int(camera_cfg["index"])
        self.camera = cv2.VideoCapture(index)

        if not self.camera.isOpened():
            print("Cannot access camera")
            sys.exit(0)
        print("Cam opened")
        fourcc = str(camera_cfg["fourcc"])
        if len(fourcc) != 4:
            print(f"ERROR: camera.fourcc must be 4 characters, got: '{fourcc}'")
            sys.exit(1)

        self.camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, int(camera_cfg["frame_width"]))
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, int(camera_cfg["frame_height"]))

        autofocus = bool(camera_cfg["autofocus"])
        focus = int(camera_cfg["focus"])  # min 0, max 255, increments of 5
        self.camera.set(cv2.CAP_PROP_AUTOFOCUS, 1 if autofocus else 0)
        if not autofocus:
            self.camera.set(cv2.CAP_PROP_FOCUS, focus)

    def get_frame(self):
        ret, frame = self.camera.read()
        return frame

class Tag:
    def __init__(self, id, raw_tag):
        self.id = id
        self.raw_tag = np.array(raw_tag)[0]

        self.center = np.sum(self.raw_tag, 0) / 4
        self.front = np.sum(self.raw_tag[:2], 0) / 2

class Robot:
    def __init__(self, tag, position, angle):
        self.tag = tag
        self.id = int(tag.id)
        self.position = position
        self.angle = angle
    
    def to_dict(self):
        return {
            "position": self.position.tolist(),  # Convert NumPy array to list
            "angle": self.angle
        }

class Tracker(threading.Thread):
    def __init__(self, cfg: dict, H: np.ndarray):
        super().__init__()
        self.cfg = cfg
        self.camera = Camera(_get_required(cfg, "camera"))
        self.calibrated = False
        self.num_corners = 0
        self.min_x = 0
        self.max_x = 0
        self.min_y = 0
        self.max_y = 0
        self.center = np.array((0,0))
        self.corner_distance_meters = 0.55
        self.corner_distance_pixels = 0
        self.num_corner_tags = 0
        self.scale_factor = 0
        self.robots = {}
        self.valid_robot_ids = set(int(k) for k in cfg.get("robots", {}).keys())
        self._prev_poses = {}  # {robot_id: (x, y, angle)} for jump detection
        self.MAX_JUMP_M = 0.3   # max plausible displacement per frame (meters)
        self.MAX_JUMP_DEG = 60   # max plausible angle change per frame (degrees)

        self.visualize = bool(_get_required(cfg, "visualization.enabled"))
        self.H = H

        board_width = float(_get_required(cfg, "arena.width_m"))   # meters
        board_height = float(_get_required(cfg, "arena.height_m"))  # meters
        self.board_height = board_height
        scale = int(_get_required(cfg, "visualization.warp_scale"))

        self.output_size = (int(board_width * scale), int(board_height * scale))
        
        dst_points = np.array([
            [0, self.output_size[1] - 1],
            [self.output_size[0] - 1, self.output_size[1] - 1],
            [self.output_size[0] - 1, 0],
            [0, 0],
        ], dtype=np.float32)

        world_points = np.array([
            [0, 0],
            [board_width, 0],
            [board_width, board_height],
            [0, board_height]
        ], dtype=np.float32)

        H_warp = cv2.getPerspectiveTransform(world_points, dst_points)
        self.H_total = H_warp @ self.H
        #warped = cv2.warpPerspective(img, H_total, output_size)

        aruco_dict_name = str(cfg.get("aruco", {}).get("dictionary", "DICT_5X5_1000"))
        aruco_enum = getattr(cv2.aruco, aruco_dict_name, None)
        if aruco_enum is None:
            raise ValueError(f"Unknown ArUco dictionary: '{aruco_dict_name}'. "
                             f"Use e.g. DICT_4X4_100, DICT_5X5_1000, DICT_6X6_250.")
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(aruco_enum)
        self.aruco_params = cv2.aruco.DetectorParameters()

        # Tune detection for small markers / difficult conditions
        self.aruco_params.adaptiveThreshWinSizeMin = 3
        self.aruco_params.adaptiveThreshWinSizeMax = 23
        self.aruco_params.adaptiveThreshWinSizeStep = 10
        self.aruco_params.adaptiveThreshConstant = 7
        self.aruco_params.minMarkerPerimeterRate = 0.01   # allow smaller markers in frame
        self.aruco_params.maxMarkerPerimeterRate = 4.0
        self.aruco_params.polygonalApproxAccuracyRate = 0.05
        self.aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX

        self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

        broker = str(_get_required(cfg, "mqtt.broker"))
        port = int(_get_required(cfg, "mqtt.port"))
        self.TOPIC = str(_get_required(cfg, "mqtt.topic"))

        self.red = (0, 0, 255)
        self.green = (0, 255, 0)
        self.magenta = (255, 0, 255)
        self.cyan = (255, 255, 0)
        self.black = (0, 0, 0)
        self.white = (255, 255, 255)

        self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        self.client.connect(broker, port, 60)
        self.client.loop_start()

        self._t_loop = None  # for timing

        # CSV pose logging
        log_dir = "logs"
        os.makedirs(log_dir, exist_ok=True)
        log_filename = os.path.join(log_dir, f"poses_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv")
        self._csv_file = open(log_filename, "w", newline="")
        self._csv_writer = csv.writer(self._csv_file)
        self._csv_writer.writerow(["timestamp", "robot_id", "x", "y", "angle", "qx", "qy", "qz", "qw", "detected"])
        print(f"Logging poses to {log_filename}")

    def homography_to_world(self, pt):
            px = np.array([[pt[0], pt[1], 1]], dtype=np.float32).T
            world_pt = self.H @ px
            world_pt /= world_pt[2]
            return world_pt[0:2].flatten()

    def point_in_polygon(self,x, y, points):
        inside = False
        n = len(points)
        for i in range(n):
            j = (i - 1) % n
            xi, yi = points[i]["x"], points[i]["y"]
            xj, yj = points[j]["x"], points[j]["y"]
            if ((yi > y) != (yj > y)) and (x < (xj - xi) * (y - yi) / (yj - yi + 1e-10) + xi):
                inside = not inside
        return inside

    def point_in_circle(self, x, y, center, radius):
        dx = x - center["x"]
        dy = y - center["y"]
        return dx * dx + dy * dy <= radius * radius

    def run(self):
        if(self.H is None): #we are not using homography to calibrate
            # calibrate the captured arena
            while not self.calibrated:
                image = self.camera.get_frame()
                overlay = image.copy()

                (corners, tag_ids, rejected) = self.aruco_detector.detectMarkers(image)
                if(self.visualize):
                    cv2.aruco.drawDetectedMarkers(overlay, corners)
                    cv2.namedWindow("Calibration", cv2.WINDOW_NORMAL)
                    cv2.imshow("Calibration", overlay)
                    cv2.waitKey(1)
                if tag_ids is not None and len(tag_ids) > 0:

                    tag_ids = np.array(tag_ids).reshape(-1)

                    # Process raw ArUco output
                    for id, raw_tag in zip(tag_ids, corners):

                        tag = Tag(id, raw_tag)                    
                        
                        if tag.id == 0: # Reserved tag ID for corners

                            if self.num_corner_tags == 0: # Record the first corner tag detected
                                self.min_x = tag.center[0]
                                self.max_x = tag.center[0]
                                self.min_y = tag.center[1]
                                self.max_y = tag.center[1]
                            else: # Set min/max boundaries of arena based on second corner tag detected

                                if tag.center[0] < self.min_x:
                                    self.min_x = tag.center[0]
                                if tag.center[0] > self.max_x:
                                    self.max_x = tag.center[0]
                                if tag.center[1] < self.min_y:
                                    self.min_y = tag.center[1]
                                if tag.center[1] > self.max_y:
                                    self.max_y = tag.center[1]

                                self.corner_distance_pixels = math.dist([self.min_x, self.min_y], [self.max_x, self.max_y]) # Euclidean distance between corner tags in pixels
                                self.scale_factor = self.corner_distance_pixels / self.corner_distance_meters
                                print("Calibrated: Scale factor: ", self.scale_factor)
                                x = ((self.max_x - self.min_x) / 2) / self.scale_factor # Convert to metres
                                y = ((self.max_y - self.min_y) / 2) / self.scale_factor # Convert to metres
                                self.center = np.array(x, y)
                                self.min_x = int(self.min_x)
                                self.min_y = int(self.min_y)
                                self.max_x = int(self.max_x)
                                self.max_y = int(self.max_y)
                                self.calibrated = True
                            
                            self.num_corner_tags = self.num_corner_tags + 1

        # Endless loop to track robots
        while True:
            t0 = time.perf_counter()
            image = self.camera.get_frame()
            t1 = time.perf_counter()
            if(image is None):
                continue
            overlay = image.copy()

            (corners, tag_ids, rejected) = self.aruco_detector.detectMarkers(image)
            t2 = time.perf_counter()
            if(self.visualize):
                cv2.aruco.drawDetectedMarkers(overlay, corners)

            self.robots = {} # Clear robots in case a robot left arena

            # Check whether any tags were detected in this camera frame
            if tag_ids is not None and len(tag_ids) > 0:

                tag_ids = np.array(tag_ids).reshape(-1)

                # Process raw ArUco output
                for id, raw_tag in zip(tag_ids, corners):
                    interval = f"  loop={1000*(t0-self._t_loop):.1f}ms" if self._t_loop else ""
                    print(f"Found ID {id}  capture={1000*(t1-t0):.1f}ms  detect={1000*(t2-t1):.1f}ms{interval}")
                    if int(id) in self.valid_robot_ids:
                        tag = Tag(id, raw_tag)
                        if(self.H is None):
                            cx = tag.center[0] / self.scale_factor
                            cy = tag.center[1] / self.scale_factor
                            position = np.array([cx, cy])
                            dx = (tag.front[0] - tag.center[0]) / self.scale_factor
                            dy = -((tag.front[1] - tag.center[1]) / self.scale_factor)
                            angle = math.degrees(math.atan2(dy, dx))
                        else:
                            center_world = self.homography_to_world(tag.center)
                            front_world = self.homography_to_world(tag.front)
                            # H already gives x-right, y-up (origin at bottom-left)
                            position = center_world
                            dx = front_world[0] - center_world[0]
                            dy = front_world[1] - center_world[1]
                            angle = math.degrees(math.atan2(dy, dx))
                        
                        rid = int(id)
                        # Jump detection: reject measurement if it jumps too far from previous
                        prev = self._prev_poses.get(rid)
                        if prev is not None:
                            dp = math.hypot(position[0] - prev[0], position[1] - prev[1])
                            da = abs(angle - prev[2])
                            if da > 180:
                                da = 360 - da
                            if dp > self.MAX_JUMP_M or da > self.MAX_JUMP_DEG:
                                print(f"JUMP REJECTED robot {rid}: dp={dp:.3f}m da={da:.1f}° "
                                      f"({prev[0]:.3f},{prev[1]:.3f},{prev[2]:.1f}) -> ({position[0]:.3f},{position[1]:.3f},{angle:.1f})")
                                continue
                        self._prev_poses[rid] = (position[0], position[1], angle)
                        self.robots[rid] = Robot(tag, position, angle)

                if(self.visualize):
                    # Draw boundary of virtual environment based on corner tag positions
                    cv2.rectangle(image, (self.min_x, self.min_y), (self.max_x, self.max_y), self.green, 1, lineType=cv2.LINE_AA)
                            
                    # Draw work and charge regions
                    cv2.rectangle(overlay, (self.min_x, self.min_y), (self.min_x + 200, self.max_y), self.green, -1, lineType=cv2.LINE_AA)
                    cv2.rectangle(overlay, (self.max_x - 200, self.min_y), (self.max_x, self.max_y), self.red, -1, lineType=cv2.LINE_AA)
            
                    # Process robots
                    for id, robot in self.robots.items():
                        # Draw tag
                        tag = robot.tag
                        
                        # Draw tag ID
                        text = str(tag.id)
                        font = cv2.FONT_HERSHEY_SIMPLEX
                        font_scale = 1.5
                        thickness = 4
                        textsize = cv2.getTextSize(text, font, font_scale, thickness)[0]
                        position = (int(tag.center[0] - textsize[0]/2), int(tag.center[1] + textsize[1]/2))
                        cv2.putText(image, text, position, font, font_scale, self.black, thickness * 3, cv2.LINE_AA)
                        cv2.putText(image, text, position, font, font_scale, self.white, thickness, cv2.LINE_AA)
                        cv2.putText(overlay, text, position, font, font_scale, self.black, thickness * 3, cv2.LINE_AA)
                        cv2.putText(overlay, text, position, font, font_scale, self.white, thickness, cv2.LINE_AA)
                                       
                    # Transparency for overlaid augments
                    alpha = 0.3
                    image = cv2.addWeighted(overlay, alpha, image, 1 - alpha, 0)
            if(self.visualize):
                warped = cv2.warpPerspective(image, self.H_total, self.output_size)
                cv2.namedWindow('Tracking System', cv2.WINDOW_NORMAL)
                cv2.imshow('Tracking System', warped)

                # Quitting with Q (necessary for fullscreen mode)
                if cv2.waitKey(1) == ord('q'):
                    sys.exit()

            # Log to CSV (log every valid robot each frame, detected or not)
            t_now = time.time()
            detected_ids = set(self.robots.keys())
            for rid in self.valid_robot_ids:
                if rid in detected_ids:
                    robot = self.robots[rid]
                    pos = robot.position
                    yaw = math.radians(robot.angle)
                    qz = math.sin(yaw / 2.0)
                    qw = math.cos(yaw / 2.0)
                    self._csv_writer.writerow([f"{t_now:.6f}", rid, f"{pos[0]:.6f}", f"{pos[1]:.6f}", f"{robot.angle:.4f}", "0.0", "0.0", f"{qz:.6f}", f"{qw:.6f}", 1])
                else:
                    self._csv_writer.writerow([f"{t_now:.6f}", rid, "", "", "", "", "", "", "", 0])
            self._csv_file.flush()

            # dump robot data to mqtt
            json_data = json.dumps({k: v.to_dict() for k, v in self.robots.items()})
            self.client.publish(self.TOPIC, json_data)
            self._t_loop = t0


if __name__ == "__main__":
    config_path = sys.argv[1] if len(sys.argv) >= 2 else DEFAULT_CONFIG_PATH
    homography_path = sys.argv[2] if len(sys.argv) >= 3 else DEFAULT_HOMOGRAPHY_PATH

    cfg = load_config(config_path)
    H = load_homography(homography_path)

    tracker = Tracker(cfg, H)
    tracker.start()
    
    
