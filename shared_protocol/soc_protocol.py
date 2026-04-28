# -*- coding: utf-8 -*-

# This file is the middleman between Matlab and the tracking code
# It decides how frames get processed and when we send back xyz points or in out calls

from .image_cache import LatestFrameStore
import cv2
import math
import numpy as np
from pathlib import Path


CMD_RESET = 1
CMD_REQUEST_LATEST_IMAGE = 10
CMD_REQUEST_NTH_PREVIOUS_IMAGE = 11
CMD_REQUEST_NTH_NEXT_IMAGE = 12
CMD_REQUEST_IMAGE_AT_FRAME = 15
CMD_LOG_DATA = 21
CMD_SEND_CALL = 22
CMD_STOP_CAPTURE = 30
CMD_PROCESS_IMAGE = 50
CMD_SLAVE_MODE = 98
CMD_SLAVE_MODE_READY = 99

# These are the two big operating modes
# Master means we keep asking for fresh live frames
# Slave means Matlab is driving replay or stepping through saved frames
MODE_MASTER = "master"
MODE_SLAVE = "slave"

# These limits keep obviously bad points from poisoning the track
# Most of them are just guard rails based on how fast and how far the ball should move
MAX_FAST_DEPTH_M = 50.0
MAX_FAST_ABS_X_M = 20.0
MAX_FAST_ABS_Y_M = 20.0
MAX_FAST_ABS_Z_M = 20.0
MAX_FAST_POSITION_JUMP_M = 5.0
MAX_FAST_X_STEP_PER_10_FRAMES_M = 2.0
MAX_FAST_Y_STEP_PER_10_FRAMES_M = 1.0
MAX_FAST_Z_STEP_PER_10_FRAMES_M = 1.0
MAX_FAST_HISTORY_GAP_FRAMES = 20
BOUNCE_Y_WINDOW_M = 0.35
BOUNCE_Y_DELTA_MIN_M = 0.03
BOUNCE_TOUCH_Y_MAX_M = 0.08

COURT_HALF_WIDTH_M = 4.115
COURT_HALF_LENGTH_M = 11.885

# These numbers convert left right pixel locations into world space
# The correction terms at the end are there because the raw stereo math was close but not quite lined up with the scene
STEREO_IMAGE_WIDTH_PX = 1920.0
STEREO_IMAGE_HEIGHT_PX = 1080.0
STEREO_PIXEL_SIZE_M = 18.75e-6
STEREO_FOCAL_LENGTH_M = 0.01
STEREO_BASELINE_M = 3.0
CAMERA_TILT_DEG = 48.0
CAMERA_HEIGHT_M = 6.5
DEPTH_REFERENCE_OFFSET_M = 9.0
DEPTH_REFERENCE_SCALE = -0.85
Y_CORR_A = 1.0116
Y_CORR_B = -0.1127
Y_CORR_C = 0.4619
Z_CORR_A = 0.0235
Z_CORR_B = 1.0423
Z_CORR_C = -0.7039

# These tune the base subtraction detector that the fast path uses
# The same general idea shows up in hardware and in the software fast fallback
FAST_NOT_FOUND_VALUE = -1.0
FAST_PRELOAD_BASE_FRAME = 40
FAST_CIRC_THRESHOLD = 50
FAST_MIN_SIZE = 8
FAST_MAX_SIZE = 45
FAST_SQUARE_TOL = 14
CV2_RED_CHANNEL_INDEX = 2
FALLBACK_FAST_CIRC_THRESHOLD = 35
FALLBACK_FAST_MIN_SIZE = 8
FALLBACK_FAST_MAX_SIZE = 45
FALLBACK_FAST_SQUARE_TOL = 14

# These are handy switches for whether a likely bounce should automatically kick us into replay handling
AUTO_ENTER_REPLAY_ON_LIKELY_OUT = False
AUTO_ENTER_REPLAY_ON_BOUNCE = False

# These are the wider hsv ranges for the pure color based fallback
# The second bright range helps when the ball gets washed out by lighting
FALLBACK_HSV_PRIMARY_LOWER = np.array([20, 45, 110], dtype=np.uint8)
FALLBACK_HSV_PRIMARY_UPPER = np.array([45, 255, 255], dtype=np.uint8)
FALLBACK_HSV_BRIGHT_LOWER = np.array([16, 20, 170], dtype=np.uint8)
FALLBACK_HSV_BRIGHT_UPPER = np.array([52, 170, 255], dtype=np.uint8)
FALLBACK_HSV_MIN_AREA = 10
FALLBACK_HSV_MIN_CIRCULARITY = 0.12
FALLBACK_HSV_TRACK_RADIUS_PX = 220.0


class SoCProtocol(object):
    # This object keeps all the runtime state for tracking and talking to Matlab
    def __init__(self, command_sender=None, fpga_cache=None, disable_hardware=False, disable_fast_path=False):
        self.mode = MODE_MASTER
        self.latest_frame = LatestFrameStore()
        self.capture_enabled = True
        self.command_sender = command_sender
        self.waiting_for_image = False
        self.fpga_cache = fpga_cache
        self.disable_hardware = disable_hardware
        self.disable_fast_path = disable_fast_path
        # These remember the recent good track so we can reject crazy jumps and spot the bounce
        self.last_good_position = None
        self.recent_positions = []
        # These hold the reference frame used by the fast subtraction path
        self.fast_base_left = None
        self.fast_base_right = None
        self.fast_base_preload_attempted = False
        self.fast_base_source = None
        self.last_bounce_reported_frame = None
        # These keep noisy hardware errors from spamming the console every frame
        self.warned_missing_fpga_cache = False
        self.seen_fpga_fast_path_errors = set()
        # These help the hsv fallback hang on to the same blob from frame to frame
        self.last_fallback_left_centroid = None
        self.last_fallback_right_centroid = None
        self.fallback_miss_count = 0

    # Everything going back to Matlab funnels through here
    def send_command(self, cmd_array):
        if self.command_sender is not None:
            self.command_sender(cmd_array)
        else:
            print("SEND:", cmd_array)

    # Ask Matlab for whatever the newest live frame is right now
    def request_latest_image(self):
        self.send_command([CMD_REQUEST_LATEST_IMAGE])

    # Ask Matlab to step backward during replay
    def request_nth_previous_image(self, offset):
        self.send_command([CMD_REQUEST_NTH_PREVIOUS_IMAGE, offset])

    # Ask Matlab to step forward during replay
    def request_nth_next_image(self, offset):
        self.send_command([CMD_REQUEST_NTH_NEXT_IMAGE, offset])

    # Ask Matlab for one exact frame when we already know the number we want
    def request_image_at_frame(self, frame_number):
        self.send_command([CMD_REQUEST_IMAGE_AT_FRAME, frame_number])

    # This is the normal heartbeat in master mode
    # If we are live and not already waiting on a frame then ask for the next one
    def drive(self):
        if self.mode == MODE_MASTER and self.capture_enabled and not self.waiting_for_image:
            self.request_latest_image()
            self.waiting_for_image = True

    # Send the final xyz point back to Matlab so it can plot and display it
    def log_position(self, frame_number, x, y, z):
        self.send_command([CMD_LOG_DATA, frame_number, x, y, z])

    # Send the final in out call back to Matlab
    # We include the bounce frame when we know it so the GUI can point to the right moment
    def send_in_out_call(self, is_in, bounce_frame=None):
        if is_in:
            value = 1
        else:
            value = 0
        label = "IN" if value == 1 else "OUT"
        if bounce_frame is None:
            print("[BOUNCE] sending %s call" % label)
        else:
            print("[BOUNCE] sending %s call for frame %s" % (label, bounce_frame))
        if bounce_frame is None:
            self.send_command([CMD_SEND_CALL, value])
        else:
            self.send_command([CMD_SEND_CALL, value, int(bounce_frame)])

    # Stop pulling new live frames and switch over to replay style control
    def stop_capture(self):
        self.capture_enabled = False
        self.waiting_for_image = False
        self.send_command([CMD_STOP_CAPTURE])
        self.enter_slave_mode()

    # Let Matlab know replay mode is ready on our side
    def send_slave_mode_ready(self):
        self.send_command([CMD_SLAVE_MODE_READY])

    # Clear anything that depends on old motion history
    # We call this when modes change or when the base frame changes enough that old positions are not trustworthy anymore
    def reset_position_history(self):
        self.last_good_position = None
        self.recent_positions = []
        self.last_bounce_reported_frame = None
        self.last_fallback_left_centroid = None
        self.last_fallback_right_centroid = None
        self.fallback_miss_count = 0

    # This is the sanity filter for xyz points
    # If a point jumps too far or lands somewhere physically silly we throw it away before it can affect bounce logic
    def is_position_reasonable(self, frame_number, x, y, z):
        if x == FAST_NOT_FOUND_VALUE and y == FAST_NOT_FOUND_VALUE and z == FAST_NOT_FOUND_VALUE:
            return False

        abs_z = abs(z)
        if abs_z > MAX_FAST_ABS_Z_M:
            return False
        if abs(x) > MAX_FAST_ABS_X_M or abs(y) > MAX_FAST_ABS_Y_M:
            return False

        # In replay mode we relax the motion history gate because Matlab may jump around between frames
        if self.mode == MODE_SLAVE:
            return True

        if self.last_good_position is not None:
            frame_gap = frame_number - self.last_good_position["frame_number"]
            if frame_gap <= 0:
                frame_gap = 10
            frame_scale = max(1.0, frame_gap / 10.0)

            dx = x - self.last_good_position["x"]
            dy = y - self.last_good_position["y"]
            dz = z - self.last_good_position["z"]
            jump = (dx * dx + dy * dy + dz * dz) ** 0.5
            if jump > MAX_FAST_POSITION_JUMP_M:
                return False
            if frame_gap <= MAX_FAST_HISTORY_GAP_FRAMES:
                if abs(dx) > MAX_FAST_X_STEP_PER_10_FRAMES_M * frame_scale:
                    return False
                if abs(dy) > MAX_FAST_Y_STEP_PER_10_FRAMES_M * frame_scale:
                    return False
                if abs(dz) > MAX_FAST_Z_STEP_PER_10_FRAMES_M * frame_scale:
                    return False

        return True

    # This stays separate mostly so the fast path can call a clearly named hook
    def is_fast_position_reasonable(self, frame_number, x, y, z):
        return self.is_position_reasonable(frame_number, x, y, z)

    # Look at the last few good points and try to spot the bottom of the bounce
    # We want to see the ball coming down then turning back up near the court plane
    def find_recent_bounce_candidate(self, frame_number, x, y, z):
        if y == FAST_NOT_FOUND_VALUE or len(self.recent_positions) < 2:
            return None
        window = [
            {"frame_number": point["frame_number"], "x": point["x"], "y": point["y"], "z": point["z"]}
            for point in self.recent_positions[-3:]
        ]
        window.append({"frame_number": frame_number, "x": x, "y": y, "z": z})

        for idx in range(1, len(window)):
            if (window[idx]["frame_number"] - window[idx - 1]["frame_number"]) > MAX_FAST_HISTORY_GAP_FRAMES:
                return None

        min_index = min(range(len(window)), key=lambda idx: window[idx]["y"])
        bounce = window[min_index]
        near_court = abs(bounce["y"]) <= BOUNCE_Y_WINDOW_M
        has_descent_before = any(
            (point["y"] - bounce["y"]) >= BOUNCE_Y_DELTA_MIN_M
            for point in window[:min_index]
        )
        has_rise_after = any(
            (point["y"] - bounce["y"]) >= BOUNCE_Y_DELTA_MIN_M
            for point in window[min_index + 1:]
        )
        # Perfect data would cross y equals zero but the hardware path can hover just above it
        # So we also accept a point that gets very close to the floor
        crosses_court_plane = any(point["y"] <= 0 for point in window) and any(point["y"] >= 0 for point in window)
        if not crosses_court_plane:
            # When the track bottoms out just above zero the hardware path can mark the trough a frame early
            # In that case prefer the latest point that still sits close to the court so the bounce location lines up with the visible contact better
            near_plane_points = [
                point for point in window[min_index:]
                if point["y"] <= BOUNCE_TOUCH_Y_MAX_M
            ]
            if near_plane_points:
                bounce = near_plane_points[-1]
                near_court = abs(bounce["y"]) <= BOUNCE_Y_WINDOW_M
                has_descent_before = any(
                    (point["y"] - bounce["y"]) >= BOUNCE_Y_DELTA_MIN_M
                    for point in window[:window.index(bounce)]
                )
                has_rise_after = any(
                    (point["y"] - bounce["y"]) >= BOUNCE_Y_DELTA_MIN_M
                    for point in window[window.index(bounce) + 1:]
                )
        touches_court_plane = crosses_court_plane or bounce["y"] <= BOUNCE_TOUCH_Y_MAX_M

        if not (near_court and has_descent_before and has_rise_after and touches_court_plane):
            return None

        return bounce

    def is_bounce_in_bounds(self, x, z):
        # Keep this check centered on the tracked bounce point so hardware and software make the same line call
        within_length = abs(x) <= COURT_HALF_LENGTH_M
        within_width = abs(z) <= COURT_HALF_WIDTH_M
        return within_length and within_width

    # Keep a short recent trail so bounce detection has something to work with
    def remember_good_fast_position(self, frame_number, x, y, z):
        position = {
            "frame_number": frame_number,
            "x": x,
            "y": y,
            "z": z
        }
        self.last_good_position = position
        self.recent_positions.append(position)
        if len(self.recent_positions) > 5:
            self.recent_positions = self.recent_positions[-5:]

    # This is the last cleanup step before we ship a result back out
    # It decides whether the point is usable and whether the recent motion looks like a bounce
    def annotate_final_result(self, result):
        x = result.get("x", FAST_NOT_FOUND_VALUE)
        y = result.get("y", FAST_NOT_FOUND_VALUE)
        z = result.get("z", FAST_NOT_FOUND_VALUE)
        frame_number = result.get("frame_number", 0)
        result["bounce"] = None

        if x == FAST_NOT_FOUND_VALUE and y == FAST_NOT_FOUND_VALUE and z == FAST_NOT_FOUND_VALUE:
            result["reasonable"] = False
            result["likely_out"] = False
            return result

        final_reasonable = bool(result.get("reasonable", False)) and self.is_position_reasonable(frame_number, x, y, z)
        result["reasonable"] = final_reasonable
        if not final_reasonable:
            result["x"] = FAST_NOT_FOUND_VALUE
            result["y"] = FAST_NOT_FOUND_VALUE
            result["z"] = FAST_NOT_FOUND_VALUE
            result["likely_out"] = False
            return result

        bounce = self.find_recent_bounce_candidate(frame_number, x, y, z)
        result["bounce"] = bounce
        result["likely_out"] = bounce is not None
        if final_reasonable:
            self.remember_good_fast_position(frame_number, x, y, z)
        return result

    # Handy helper for the many ways detection can fail
    def build_empty_fast_result(self, frame_number, reasonable=False):
        return {
            "frame_number": frame_number,
            "x": FAST_NOT_FOUND_VALUE,
            "y": FAST_NOT_FOUND_VALUE,
            "z": FAST_NOT_FOUND_VALUE,
            "reasonable": reasonable,
            "likely_out": False,
        }

    # The fast contour code returns a bounding box and we just use its center
    def center_from_fast_match(self, match):
        if match is None:
            return None
        bb = match[0]
        return (
            bb[0] + (bb[2] / 2.0),
            bb[1] + (bb[3] / 2.0),
        )

    # The tuned subtraction path works off the live red channel
    # Matlab sends rgb so channel zero here is the red plane we want
    def extract_fast_channel(self, image):
        image = np.asarray(image, dtype=np.uint8)
        if image.ndim >= 3:
            return np.ascontiguousarray(image[:, :, 0], dtype=np.uint8)
        return np.ascontiguousarray(image, dtype=np.uint8)

    # Search the repo for frame 40 stereo pairs and pick the one that best matches the current scene
    def select_preload_fast_base_images(self, left_reference=None, right_reference=None):
        repo_root = Path(__file__).resolve().parents[1]
        candidate_pairs = []
        search_roots = [repo_root]
        for child in sorted(repo_root.iterdir()):
            if child.is_dir():
                search_roots.append(child)

        for folder in search_roots:
            for ext in (".jpg", ".jpeg", ".png"):
                candidate_left = folder / ("LeftFrame_%d%s" % (FAST_PRELOAD_BASE_FRAME, ext))
                candidate_right = folder / ("RightFrame_%d%s" % (FAST_PRELOAD_BASE_FRAME, ext))
                if candidate_left.is_file() and candidate_right.is_file():
                    candidate_pairs.append((folder, candidate_left, candidate_right))
                    break

        if not candidate_pairs:
            return None

        if left_reference is None or right_reference is None:
            folder, left_path, right_path = candidate_pairs[0]
        else:
            left_reference = np.asarray(left_reference, dtype=np.uint8)
            right_reference = np.asarray(right_reference, dtype=np.uint8)
            left_reference = left_reference[::8, ::8]
            right_reference = right_reference[::8, ::8]
            best_choice = None
            best_score = None
            for folder, left_path, right_path in candidate_pairs:
                left_image = cv2.imread(str(left_path), cv2.IMREAD_COLOR)
                right_image = cv2.imread(str(right_path), cv2.IMREAD_COLOR)
                if left_image is None or right_image is None:
                    continue
                left_candidate = np.ascontiguousarray(left_image[:, :, CV2_RED_CHANNEL_INDEX], dtype=np.uint8)[::8, ::8]
                right_candidate = np.ascontiguousarray(right_image[:, :, CV2_RED_CHANNEL_INDEX], dtype=np.uint8)[::8, ::8]
                score = float(np.mean(np.abs(left_reference.astype(np.int16) - left_candidate.astype(np.int16))))
                score += float(np.mean(np.abs(right_reference.astype(np.int16) - right_candidate.astype(np.int16))))
                if best_score is None or score < best_score:
                    best_score = score
                    best_choice = (folder, left_path, right_path)
            if best_choice is None:
                return None
            folder, left_path, right_path = best_choice

        left_image = cv2.imread(str(left_path), cv2.IMREAD_COLOR)
        right_image = cv2.imread(str(right_path), cv2.IMREAD_COLOR)
        if left_image is None or right_image is None:
            return None

        left_base = np.ascontiguousarray(left_image[:, :, CV2_RED_CHANNEL_INDEX], dtype=np.uint8)
        right_base = np.ascontiguousarray(right_image[:, :, CV2_RED_CHANNEL_INDEX], dtype=np.uint8)
        return folder, left_base, right_base

    # Try to grab a matching frame 40 stereo base from disk
    # This keeps the fast path aligned with the current dataset instead of hardwiring serve1
    def try_preload_fast_base_images(self, left_reference=None, right_reference=None):
        if self.fast_base_left is not None and self.fast_base_right is not None:
            return True
        if self.fast_base_preload_attempted and left_reference is None and right_reference is None:
            return False
        choice = self.select_preload_fast_base_images(left_reference=left_reference, right_reference=right_reference)
        self.fast_base_preload_attempted = True
        if choice is None:
            return False
        folder, left_base, right_base = choice
        self.fast_base_left = left_base
        self.fast_base_right = right_base
        self.fast_base_source = str(folder)
        return True

    # Take a binary mask and look for something that feels like the ball
    # The checks here are intentionally simple because this code runs a lot
    def detect_fast_match_from_bw_with_params(self, bw, circ_threshold, min_size, max_size, square_tol):
        bw = np.asarray(bw, dtype=np.uint8)

        # Close small holes so the contour does not get split up into junk
        bw = cv2.morphologyEx(bw, cv2.MORPH_CLOSE, np.ones((10, 10), np.uint8))

        # Pull out contours and boxes so we can sort candidate blobs by size
        contour_result = cv2.findContours(bw, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contours = contour_result[-2]
        contours_poly = [None] * len(contours)
        bound_rects = [None] * len(contours)
        for i, contour in enumerate(contours):
            contours_poly[i] = cv2.approxPolyDP(contour, 3, True)
            bound_rects[i] = cv2.boundingRect(contours_poly[i])

        del contours_poly
        del bound_rects

        # Bigger blobs are more interesting so we try them first
        contour_data = []
        for contour in contours:
            bb = cv2.boundingRect(contour)
            contour_data.append((contour, bb))
        contour_data = sorted(contour_data, key=lambda x: x[1][2], reverse=True)

        # Walk the candidates until one looks enough like the ball
        matches = []
        for contour, bb in contour_data:
            # First pass is just shape and rough location
            if (
                abs(bb[2] - bb[3]) <= square_tol and  # the ball blob should be close to square
                bb[2] >= min_size and bb[2] <= max_size and  # too small is noise and too big is probably not the ball
                bb[3] >= min_size and bb[3] <= max_size and
                bb[1] > 100  # keep the bright junk near the lights out of the game
            ):
                # Then we check how circle like the contour is
                area = cv2.contourArea(contour)
                perimeter = cv2.arcLength(contour, True)

                if perimeter == 0:
                    continue

                # This is just a cheap integer version of circularity
                circularity = (1257 * area // pow(perimeter, 2))
                if circularity > circ_threshold:
                    matches.append((bb, circularity))
                    break

            # Once the sorted boxes are tiny the rest are not worth checking
            elif bb[2] < min_size and bb[3] < min_size:
                break

        if not matches:
            return None

        return matches[0]

    # Main fast path contour settings
    def detect_fast_match_from_bw(self, bw):
        return self.detect_fast_match_from_bw_with_params(
            bw,
            FAST_CIRC_THRESHOLD,
            FAST_MIN_SIZE,
            FAST_MAX_SIZE,
            FAST_SQUARE_TOL,
        )

    # This is the main software version of the fast detector
    # Subtract the base frame then threshold the changed pixels
    def detect_fast_match_from_raw(self, base, raw):
        diff = cv2.subtract(np.asarray(raw, dtype=np.uint8), np.asarray(base, dtype=np.uint8))
        _, bw = cv2.threshold(diff, 25, 255, cv2.THRESH_BINARY)
        return self.detect_fast_match_from_bw(bw)

    # Same idea as the fast path but a little more forgiving when we are trying to rescue a bad frame
    def detect_fallback_fast_match_from_raw(self, base, raw):
        diff = cv2.subtract(np.asarray(raw, dtype=np.uint8), np.asarray(base, dtype=np.uint8))
        _, bw = cv2.threshold(diff, 25, 255, cv2.THRESH_BINARY)
        return self.detect_fast_match_from_bw_with_params(
            bw,
            FALLBACK_FAST_CIRC_THRESHOLD,
            FALLBACK_FAST_MIN_SIZE,
            FALLBACK_FAST_MAX_SIZE,
            FALLBACK_FAST_SQUARE_TOL,
        )

    # Turn left right pixel centers into world coordinates
    # The raw stereo math gets us close and the correction terms clean up the last bit of scene specific error
    def stereo_pixels_to_corrected_world(self, x_left, y_left, x_right, y_right):
        xCenter = STEREO_IMAGE_WIDTH_PX / 2.0
        yCenter = STEREO_IMAGE_HEIGHT_PX / 2.0
        b = STEREO_BASELINE_M
        ps = STEREO_PIXEL_SIZE_M
        f = STEREO_FOCAL_LENGTH_M

        xLeft = x_left
        xRight = x_right
        yLeft = y_left
        yRight = yLeft

        # More left right separation means the ball is closer
        d = (abs((xLeft - xCenter) - (xRight - xCenter)) * ps)
        if d <= ps:
            return None

        # This is the straight stereo triangulation part
        Z = (b * f) / d
        Y = Z * ((((yLeft + yRight) / 2.0) - yCenter) * ps / f)
        X = Z * ((((xLeft + xRight) / 2.0) - xCenter) * ps / f)

        X_corr = X
        # The cameras are tilted so we rotate the raw geometry into something more court aligned
        Y_corr_bad = (CAMERA_HEIGHT_M - ((Z * math.sin(math.radians(CAMERA_TILT_DEG))) + (Y * math.cos(math.radians(CAMERA_TILT_DEG)))))
        Z_corr_bad = DEPTH_REFERENCE_SCALE * ((Z * math.cos(math.radians(CAMERA_TILT_DEG)) - Y * math.sin(math.radians(CAMERA_TILT_DEG))) - DEPTH_REFERENCE_OFFSET_M)

        # Final regression tweak from calibration work
        Y_corr = Y_CORR_A * Y_corr_bad + Y_CORR_B * Z_corr_bad + Y_CORR_C
        Z_corr = Z_CORR_A * Y_corr_bad + Z_CORR_B * Z_corr_bad + Z_CORR_C

        return X_corr, Y_corr, Z_corr

    # Take the matched blobs from both cameras and turn them into one world point
    def fast_matches_to_world(self, frame_number, left_match, right_match):
        left_center = self.center_from_fast_match(left_match)
        right_center = self.center_from_fast_match(right_match)

        if left_center is None or right_center is None:
            return self.build_empty_fast_result(frame_number, reasonable=False)

        u_left, v_left = left_center
        u_right, v_right = right_center
        corrected = self.stereo_pixels_to_corrected_world(u_left, v_left, u_right, v_right)
        if corrected is None:
            return self.build_empty_fast_result(frame_number, reasonable=False)
        x, y, z = corrected

        reasonable = self.is_fast_position_reasonable(frame_number, x, y, z)

        return {
            "frame_number": frame_number,
            "x": x,
            "y": y,
            "z": z,
            "reasonable": reasonable,
            "likely_out": False,
        }

    # This is for the future version where hardware returns real candidate registers
    # Right now the custom IP still zeros those fields so this path usually does not carry the whole run
    def fpga_candidate_to_world(self, frame_number, result):
        x_left = float(result.get("result_x", FAST_NOT_FOUND_VALUE))
        y_left = float(result.get("result_y", FAST_NOT_FOUND_VALUE))
        x_right = float(result.get("result_z", FAST_NOT_FOUND_VALUE))

        if not np.isfinite(x_left) or not np.isfinite(y_left) or not np.isfinite(x_right):
            return self.build_empty_fast_result(frame_number, reasonable=False)

        corrected = self.stereo_pixels_to_corrected_world(x_left, y_left, x_right, y_left)
        if corrected is None:
            return self.build_empty_fast_result(frame_number, reasonable=False)

        x, y, z = corrected
        reasonable = self.is_fast_position_reasonable(frame_number, x, y, z)
        return {
            "frame_number": frame_number,
            "x": x,
            "y": y,
            "z": z,
            "reasonable": reasonable,
            "likely_out": False,
        }

    # Build the color mask for the pure hsv fallback
    # We combine two ranges because the ball can look different depending on lighting and distance
    def build_hsv_fallback_mask(self, image):
        hsv = cv2.cvtColor(np.asarray(image, dtype=np.uint8), cv2.COLOR_RGB2HSV)
        primary_mask = cv2.inRange(hsv, FALLBACK_HSV_PRIMARY_LOWER, FALLBACK_HSV_PRIMARY_UPPER)
        bright_mask = cv2.inRange(hsv, FALLBACK_HSV_BRIGHT_LOWER, FALLBACK_HSV_BRIGHT_UPPER)
        mask = cv2.bitwise_or(primary_mask, bright_mask)
        mask[:101, :] = 0
        return mask

    # Score hsv blobs and prefer the one that looks both round and close to where the last one was
    def centroid_from_hsv_fallback_mask(
        self,
        mask,
        previous_centroid=None,
    ):
        mask = np.asarray(mask, dtype=np.uint8)
        open_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        close_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, open_kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, close_kernel)

        contour_result = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = contour_result[-2]
        best_centroid = None
        best_score = -1.0
        for contour in contours:
            area = cv2.contourArea(contour)
            perimeter = cv2.arcLength(contour, True)
            if area < FALLBACK_HSV_MIN_AREA or perimeter == 0:
                continue

            moments = cv2.moments(contour)
            if moments["m00"] == 0:
                continue

            centroid = (
                moments["m10"] / moments["m00"],
                moments["m01"] / moments["m00"],
            )
            if centroid[1] <= 100:
                continue

            circularity = (4.0 * np.pi * area) / (perimeter ** 2)
            if circularity < FALLBACK_HSV_MIN_CIRCULARITY:
                continue

            area_score = min(area / 250.0, 1.0)
            proximity_score = 0.0
            if previous_centroid is not None:
                distance = math.hypot(
                    centroid[0] - previous_centroid[0],
                    centroid[1] - previous_centroid[1],
                )
                proximity_score = max(0.0, 1.0 - (distance / FALLBACK_HSV_TRACK_RADIUS_PX))
            # This weighted score is just a simple way to balance shape size and continuity
            score = (0.65 * min(circularity, 1.0)) + (0.20 * area_score) + (0.15 * proximity_score)

            if score > best_score:
                best_score = score
                best_centroid = centroid

        return best_centroid

    # Last ditch centroid helper when contour logic gives up but the mask still has nonzero pixels
    def centroid_from_nonzero_pixels(self, mask):
        mask = np.asarray(mask, dtype=np.uint8)
        ys, xs = np.nonzero(mask)
        if xs.size == 0 or ys.size == 0:
            return None
        return (float(np.mean(xs)), float(np.mean(ys)))

    # Prefer the biggest connected blob before falling back to a raw average of all nonzero pixels
    def centroid_from_largest_component(self, mask, min_area=20):
        binary = (np.asarray(mask, dtype=np.uint8) > 0).astype(np.uint8)
        component_count, labels, stats, centroids = cv2.connectedComponentsWithStats(binary, 8)
        best_area = 0
        best_centroid = None
        for component_index in range(1, component_count):
            area = int(stats[component_index, cv2.CC_STAT_AREA])
            if area < min_area or area <= best_area:
                continue
            best_area = area
            best_centroid = (
                float(centroids[component_index][0]),
                float(centroids[component_index][1]),
            )
        return best_centroid

    # Turn hardware masks back into a world point when the custom IP did not give us usable xyz registers
    def fpga_masks_to_world(self, frame_number, left_mask, right_mask):
        left_centroid = self.centroid_from_largest_component(left_mask)
        right_centroid = self.centroid_from_largest_component(right_mask)
        if left_centroid is None or right_centroid is None:
            left_centroid = self.centroid_from_nonzero_pixels(left_mask)
            right_centroid = self.centroid_from_nonzero_pixels(right_mask)
        if left_centroid is None or right_centroid is None:
            return self.build_empty_fast_result(frame_number, reasonable=False)

        u_left, v_left = left_centroid
        u_right, v_right = right_centroid
        corrected = self.stereo_pixels_to_corrected_world(u_left, v_left, u_right, v_right)
        if corrected is None:
            return self.build_empty_fast_result(frame_number, reasonable=False)

        x, y, z = corrected
        reasonable = self.is_fast_position_reasonable(frame_number, x, y, z)
        return {
            "frame_number": frame_number,
            "x": x,
            "y": y,
            "z": z,
            "reasonable": reasonable,
            "likely_out": False,
        }

    # This is the mixed hardware software path
    # The fpga makes the masks fast and python still finishes the centroid and world conversion work
    def fast_process_image(self, frame_number, image_data):
        if self.fpga_cache is None:
            raise RuntimeError("fpga_cache must be configured with a DMA backed PingPongFpgaCache")

        self.fpga_cache.submit_frame(frame_number=frame_number, image_data=image_data)
        result = self.fpga_cache.read_result()

        if result.get("base_updated", False):
            self.reset_position_history()
            return {
                "frame_number": frame_number,
                "x": FAST_NOT_FOUND_VALUE,
                "y": FAST_NOT_FOUND_VALUE,
                "z": FAST_NOT_FOUND_VALUE,
                "reasonable": True,
                "likely_out": False,
                "base_updated": True
            }

        frame_num, _, _ = self.extract_stereo_pair(frame_number, image_data)
        left_match = None
        right_match = None
        if result.get("candidate_valid", False):
            fast_result = self.fpga_candidate_to_world(frame_num, result)
        else:
            fast_result = self.build_empty_fast_result(frame_num, reasonable=False)

        # If the hardware candidate registers are not useful yet fall back to the returned masks
        if not fast_result.get("reasonable", False):
            fast_result = self.fpga_masks_to_world(frame_num, result["left_mask"], result["right_mask"])

        # Final fallback is to run the normal contour code directly on the hardware masks
        if not fast_result.get("reasonable", False):
            left_match = self.detect_fast_match_from_bw(result["left_mask"])
            right_match = self.detect_fast_match_from_bw(result["right_mask"])
            fast_result = self.fast_matches_to_world(frame_num, left_match, right_match)

        fast_result["base_updated"] = False
        return fast_result

    # This is the all software version of the fast path
    # It uses the same base subtraction idea as the hardware path but keeps everything in python and opencv
    def software_fast_process_image(self, frame_number, image_data):
        frame_num, left_image, right_image = self.extract_stereo_pair(frame_number, image_data)
        left_channel = self.extract_fast_channel(left_image)
        right_channel = self.extract_fast_channel(right_image)

        if self.fast_base_left is None or self.fast_base_right is None:
            self.try_preload_fast_base_images(left_channel, right_channel)

        if self.fast_base_left is None or self.fast_base_right is None or frame_number == 0:
            self.fast_base_left = left_channel.copy()
            self.fast_base_right = right_channel.copy()
            self.reset_position_history()
            return {
                "frame_number": frame_num,
                "x": FAST_NOT_FOUND_VALUE,
                "y": FAST_NOT_FOUND_VALUE,
                "z": FAST_NOT_FOUND_VALUE,
                "reasonable": True,
                "likely_out": False,
                "base_updated": True,
            }

        left_match = self.detect_fast_match_from_raw(self.fast_base_left, left_channel)
        right_match = self.detect_fast_match_from_raw(self.fast_base_right, right_channel)
        fast_result = self.fast_matches_to_world(frame_num, left_match, right_match)
        fast_result["base_updated"] = False
        return fast_result

    # By the time we get here Matlab should have already bundled left and right together
    def extract_stereo_pair(self, frame_number, image_data):
        if isinstance(image_data, dict) and "left_image" in image_data and "right_image" in image_data:
            return frame_number, image_data["left_image"], image_data["right_image"]

        raise ValueError("fallback_process_image expects an already-paired stereo frame")

    # This is the plain color based fallback with no base subtraction tricks
    # It exists mostly so we can compare against a totally different detection idea
    def hsv_fallback_process_image(self, frame_number, image_data):
        frame_num, left_image, right_image = self.extract_stereo_pair(frame_number, image_data)

        left_mask = self.build_hsv_fallback_mask(left_image)
        right_mask = self.build_hsv_fallback_mask(right_image)

        left_centroid = self.centroid_from_hsv_fallback_mask(left_mask, self.last_fallback_left_centroid)
        right_centroid = self.centroid_from_hsv_fallback_mask(right_mask, self.last_fallback_right_centroid)

        # If either side disappears we hold on briefly before forgetting the last centroid trail
        if left_centroid is None or right_centroid is None:
            self.fallback_miss_count += 1
            if self.fallback_miss_count >= 3:
                self.last_fallback_left_centroid = None
                self.last_fallback_right_centroid = None
            return {"frame_number": frame_num, "x": FAST_NOT_FOUND_VALUE, "y": FAST_NOT_FOUND_VALUE, "z": FAST_NOT_FOUND_VALUE, "reasonable": False, "likely_out": False}

        u_left, v_left = left_centroid
        u_right, v_right = right_centroid
        corrected = self.stereo_pixels_to_corrected_world(u_left, v_left, u_right, v_right)
        if corrected is None:
            self.fallback_miss_count += 1
            return {"frame_number": frame_num, "x": FAST_NOT_FOUND_VALUE, "y": FAST_NOT_FOUND_VALUE, "z": FAST_NOT_FOUND_VALUE, "reasonable": False, "likely_out": False}
        x, y, z = corrected
        self.last_fallback_left_centroid = left_centroid
        self.last_fallback_right_centroid = right_centroid
        self.fallback_miss_count = 0

        return {
            "frame_number": frame_num,
            "x": x,
            "y": y,
            "z": z,
            "reasonable": True,
            "likely_out": False,
        }

    # This is the normal rescue path when the main detector is unhappy
    # It first tries a looser base subtraction pass and only then drops to hsv only
    def fallback_process_image(self, frame_number, image_data):
        frame_num, left_image, right_image = self.extract_stereo_pair(frame_number, image_data)

        if self.fast_base_left is None or self.fast_base_right is None:
            self.try_preload_fast_base_images()

        if self.fast_base_left is not None and self.fast_base_right is not None:
            left_channel = self.extract_fast_channel(left_image)
            right_channel = self.extract_fast_channel(right_image)
            left_match = self.detect_fallback_fast_match_from_raw(self.fast_base_left, left_channel)
            right_match = self.detect_fallback_fast_match_from_raw(self.fast_base_right, right_channel)
            if left_match is not None and right_match is not None:
                fallback_fast_result = self.fast_matches_to_world(frame_num, left_match, right_match)
                return fallback_fast_result

        return self.hsv_fallback_process_image(frame_number, image_data)

    # Slave mode means Matlab chooses the frames instead of us constantly asking for the newest one
    def enter_slave_mode(self):
        self.mode = MODE_SLAVE
        self.waiting_for_image = False
        self.reset_position_history()
        self.send_slave_mode_ready()

    # This puts us back into the normal live tracking loop
    def reset_to_master_mode(self):
        self.mode = MODE_MASTER
        self.capture_enabled = True
        self.waiting_for_image = False
        self.reset_position_history()

    # Once we think we saw a bounce this packages up the actual in out decision
    # Right now the decision is just based on whether the bounce point lands inside the court box
    def perform_backtracking_procedure(self, frame_number, bounce=None):
        if bounce is None and len(self.recent_positions) > 0:
            candidate = self.recent_positions[-1]
            if abs(candidate["y"]) <= BOUNCE_Y_WINDOW_M:
                bounce = candidate

        if bounce is None:
            return {
                "frame_number": frame_number,
                "confirmed_out": False,
                "bounce_frame": None,
                "bounce_x": FAST_NOT_FOUND_VALUE,
                "bounce_y": FAST_NOT_FOUND_VALUE,
                "bounce_z": FAST_NOT_FOUND_VALUE,
            }

        confirmed_in = self.is_bounce_in_bounds(bounce["x"], bounce["z"])
        return {
            "frame_number": frame_number,
            "confirmed_out": not confirmed_in,
            "bounce_frame": bounce["frame_number"],
            "bounce_x": bounce["x"],
            "bounce_y": bounce["y"],
            "bounce_z": bounce["z"],
        }

    # This is the main path selector for the whole tracker
    # Depending on the flags and how the current frame behaves we pick hardware fast software fast or fallback
    def run_fast_then_fallback(self, frame_number, image_data):
        if self.disable_fast_path:
            result = self.hsv_fallback_process_image(frame_number, image_data)
            result["used_fallback"] = True
            result["used_software_fast_path"] = False
            return result

        if self.disable_hardware:
            result = self.software_fast_process_image(frame_number, image_data)
            result["used_fallback"] = False
            result["used_software_fast_path"] = True
        elif self.fpga_cache is None:
            if not self.warned_missing_fpga_cache:
                print("[FPGA] DMA cache unavailable, using software FAST")
                self.warned_missing_fpga_cache = True
            result = self.software_fast_process_image(frame_number, image_data)
            result["used_fallback"] = False
            result["used_software_fast_path"] = True
        else:
            try:
                result = self.fast_process_image(frame_number, image_data)
                result["used_fallback"] = False
                result["used_software_fast_path"] = False
            except Exception as exc:
                error_message = str(exc)
                if error_message not in self.seen_fpga_fast_path_errors:
                    print("[FPGA] Fast path failed, using CPU fallback:", exc)
                    self.seen_fpga_fast_path_errors.add(error_message)
                result = self.fallback_process_image(frame_number, image_data)
                result["used_fallback"] = True
                result["used_software_fast_path"] = False
                return result

        # Even when the first detector returns something we still kick it to fallback if the point fails sanity checks
        if not result.get("reasonable", True):
            result = self.fallback_process_image(frame_number, image_data)
            result["used_fallback"] = True
            result["used_software_fast_path"] = False

        return result

    # This handles one stereo frame all the way from detection to logging and possible bounce calls
    def handle_process_image(self, frame_number, image_data):
        self.waiting_for_image = False
        self.latest_frame.put(frame_number, image_data)

        result = self.run_fast_then_fallback(frame_number, image_data)

        result = self.annotate_final_result(result)

        # Ball Tracker print, tells us what the tracker finally believed
        print(
            "[TRACK] frame=%s x=%.4f y=%.4f z=%.4f reasonable=%s fallback=%s sw_fast=%s likely_out=%s bounce_frame=%s" % (
                result.get("frame_number", -1),
                float(result.get("x", FAST_NOT_FOUND_VALUE)),
                float(result.get("y", FAST_NOT_FOUND_VALUE)),
                float(result.get("z", FAST_NOT_FOUND_VALUE)),
                result.get("reasonable", False),
                result.get("used_fallback", False),
                result.get("used_software_fast_path", False),
                result.get("likely_out", False),
                None if result.get("bounce") is None else result["bounce"].get("frame_number"),
            )
        )

        self.log_position(
            result["frame_number"],
            result["x"],
            result["y"],
            result["z"]
        )

        # Only live master mode is allowed to send bounce decisions back to Matlab
        if self.mode == MODE_MASTER and result.get("likely_out", False):
            confirmation = self.perform_backtracking_procedure(result["frame_number"], bounce=result.get("bounce"))
            result.update(confirmation)
            bounce_frame = confirmation.get("bounce_frame")
            if bounce_frame is not None and bounce_frame != self.last_bounce_reported_frame:
                confirmed_out = confirmation.get("confirmed_out", False)
                result["confirmed_out"] = confirmed_out
                self.send_in_out_call(not confirmed_out, bounce_frame=bounce_frame)
                self.last_bounce_reported_frame = bounce_frame
                if AUTO_ENTER_REPLAY_ON_BOUNCE:
                    self.stop_capture()
            if result.get("likely_out", False) and AUTO_ENTER_REPLAY_ON_LIKELY_OUT:
                self.stop_capture()

        return result

    # This is the tiny command dispatcher that turns Matlab messages into tracker actions
    def handle_incoming_command(self, cmd_array):
        if cmd_array is None or len(cmd_array) == 0:
            raise ValueError("cmd_array must not be empty")

        cmd = cmd_array[0]

        if cmd == CMD_RESET:
            self.reset_to_master_mode()
            return {"status": "reset_to_master"}

        elif cmd == CMD_STOP_CAPTURE:
            self.capture_enabled = False
            self.waiting_for_image = False
            return {"status": "capture_stopped"}
        
        elif cmd == CMD_SLAVE_MODE:
            self.enter_slave_mode()
            return {"status": "entered_slave_mode"}
        
        elif cmd == CMD_PROCESS_IMAGE:
            if len(cmd_array) < 2:
                raise ValueError("processImage requires image_data and optional frame_number")
            if len(cmd_array) == 2:
                frame_number = None
                image_data = cmd_array[1]
            else:
                frame_number = cmd_array[1]
                image_data = cmd_array[2]
            return self.handle_process_image(frame_number, image_data)
        
        else:
            raise ValueError("Unknown command: %s" % str(cmd))
