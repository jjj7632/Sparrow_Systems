# -*- coding: utf-8 -*-

from .image_cache import LatestFrameStore
import cv2
import numpy as np


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

MODE_MASTER = "master"
MODE_SLAVE = "slave"

MAX_FAST_DEPTH_M = 50.0
MAX_FAST_ABS_X_M = 20.0
MAX_FAST_ABS_Y_M = 20.0
MAX_FAST_POSITION_JUMP_M = 5.0

COURT_HALF_WIDTH_M = 4.115
COURT_HALF_LENGTH_M = 11.885
COURT_OUT_MARGIN_M = 0.15


class SoCProtocol(object):
    # Initialize SoC mode, the capture state and the outbound commands
    def __init__(self, command_sender=None, fpga_cache=None, disable_fpga_fast_path=False):
        self.mode = MODE_MASTER
        self.latest_frame = LatestFrameStore()
        self.capture_enabled = True
        self.command_sender = command_sender
        self.waiting_for_image = False
        self.fpga_cache = fpga_cache
        self.disable_fpga_fast_path = disable_fpga_fast_path
        self.last_good_position = None

    # Send a command outward through the configured transport callback
    def send_command(self, cmd_array):
        if self.command_sender is not None:
            self.command_sender(cmd_array)
        else:
            print("SEND:", cmd_array)

    # Ask Matlab for the newest captured image
    def request_latest_image(self):
        self.send_command([CMD_REQUEST_LATEST_IMAGE])

    # Ask Matlab for an older replay frame by relative offset
    def request_nth_previous_image(self, offset):
        self.send_command([CMD_REQUEST_NTH_PREVIOUS_IMAGE, offset])

    # Ask Matlab for a newer replay frame by relative offset
    def request_nth_next_image(self, offset):
        self.send_command([CMD_REQUEST_NTH_NEXT_IMAGE, offset])

    # Ask Matlab for the frame associated with a specific frame number
    def request_image_at_frame(self, frame_number):
        self.send_command([CMD_REQUEST_IMAGE_AT_FRAME, frame_number])

    def drive(self):
        if self.mode == MODE_MASTER and self.capture_enabled and not self.waiting_for_image:
            self.request_latest_image()
            self.waiting_for_image = True

    # Return a computed XYZ position to Matlab
    def log_position(self, frame_number, x, y, z):
        self.send_command([CMD_LOG_DATA, frame_number, x, y, z])

    # Return the final in/out decision to Matlab
    def send_in_out_call(self, is_in):
        if is_in:
            value = 1
        else:
            value = 0
        self.send_command([CMD_SEND_CALL, value])

    # Tell Matlab to stop live capture before replay processing
    def stop_capture(self):
        self.capture_enabled = False
        self.waiting_for_image = False
        self.send_command([CMD_STOP_CAPTURE])
        self.enter_slave_mode()

    # Signal that the SoC is ready for slave mode replay commands
    def send_slave_mode_ready(self):
        self.send_command([CMD_SLAVE_MODE_READY])

    # Run the fast primary processing path used in master mode
    def is_fast_position_reasonable(self, x, y, z):
        if z <= 0.0 or z > MAX_FAST_DEPTH_M:
            return False
        if abs(x) > MAX_FAST_ABS_X_M or abs(y) > MAX_FAST_ABS_Y_M:
            return False

        if self.last_good_position is not None:
            dx = x - self.last_good_position["x"]
            dy = y - self.last_good_position["y"]
            dz = z - self.last_good_position["z"]
            jump = (dx * dx + dy * dy + dz * dz) ** 0.5
            if jump > MAX_FAST_POSITION_JUMP_M:
                return False

        return True

    # MVP out trigger next step #TODO is to prolly replace with camera to court transform
    def is_likely_out_fast(self, x, z):
        return (
            abs(x) > COURT_HALF_WIDTH_M + COURT_OUT_MARGIN_M or
            abs(z) > COURT_HALF_LENGTH_M + COURT_OUT_MARGIN_M
        )

    def remember_good_fast_position(self, frame_number, x, y, z):
        self.last_good_position = {
            "frame_number": frame_number,
            "x": x,
            "y": y,
            "z": z
        }

    def fast_process_image(self, frame_number, image_data):
        # Master mode fast path - 
        #  Submits the paired stereo frame to FPGAaccessible DDR buffers
        #  Lets the FPGA run the red channel subtraction/threshold candidate detector
        #  Read back image space stereo centroids and convert them to world space XYZ
        #  Mark unreliable results for fallback and suspicious results for out call confirmation
        if self.fpga_cache is None:
            raise RuntimeError("fpga_cache must be configured with a DMA backed PingPongFpgaCache")

        # Frame 0 is special, it updates the persistent base image instead of processing a shot
        self.fpga_cache.submit_frame(frame_number=frame_number, image_data=image_data)
        result = self.fpga_cache.read_result()

        # Base image updates are successful calibration events not ball detections
        if result.get("base_updated", False):
            self.last_good_position = None
            return {
                "frame_number": frame_number,
                "x": 0.0,
                "y": 0.0,
                "z": 0.0,
                "reasonable": True,
                "likely_out": False,
                "base_updated": True
            }
        
        # If the FPGA did not find a valid stereo candidate run the software fallback
        if not result.get("candidate_valid", False):
            fallback_result = self.fallback_process_image(frame_number, image_data)
            fallback_result["fpga_status"] = result.get("status", 0)
            fallback_result["fpga_candidate_valid"] = False
            print("[FPGA] No valid FPGA candidate using CPU fallback:", fallback_result)
            return fallback_result

        # The FPGA returns pixel centroids from the accelerated red channel detector
        FOCAL_PX = 10.0 / 0.006
        CX = 1920 / 2.0
        CY = 1080 / 2.0
        BASELINE = 0.10

        u_left = result["left_x"]
        v_left = result["left_y"]
        u_right = result["right_x"]
        disparity = u_left - u_right

        # Very small or negative disparity means the stereo depth estimate cannot be trusted
        if disparity <= 1.0:
            fallback_result = self.fallback_process_image(frame_number, image_data)
            fallback_result["fpga_status"] = result.get("status", 0)
            fallback_result["fpga_candidate_valid"] = True
            return fallback_result

        z = (FOCAL_PX * BASELINE) / disparity
        x = (u_left - CX) * z / FOCAL_PX
        y = (v_left - CY) * z / FOCAL_PX
        
        # Reasonable decides whether the fast result is trusted or should fall back to Python
        reasonable = self.is_fast_position_reasonable(x, y, z)

        # likely_out is only a trigger, backtracking should make the final in/out call
        likely_out = reasonable and self.is_likely_out_fast(x, z)
        if reasonable:
            self.remember_good_fast_position(frame_number, x, y, z)

        return {
            "frame_number": frame_number,
            "x": x,
            "y": y,
            "z": z,
            "reasonable": reasonable,
            "likely_out": likely_out,
            "base_updated": False
        }

    # Run the slower fallback processing path used for replay or bad calls
    def extract_stereo_pair(self, frame_number, image_data):
        if isinstance(image_data, dict) and "left_image" in image_data and "right_image" in image_data:
            return frame_number, image_data["left_image"], image_data["right_image"]

        raise ValueError("fallback_process_image expects an already-paired stereo frame")

    # Run the slower fallback processing path used for replay or bad calls
    def fallback_process_image(self, frame_number, image_data):
        # CPU fallback path - 
        #  Use color thresholding to find the ball in both images
        #  Pick the most circular contour as the ball candidate
        #  Convert the stereo pixel locations into world-space XYZ

        # Camera constants for converting stereo image space disparity into XYZ
        FOCAL_PX = 10.0 / 0.006
        CX       = 1920 / 2.0
        CY       = 1080 / 2.0
        BASELINE = 0.10

        # fallback path assumes MATLAB/PC cache already paired the left and right frames
        frame_num, left_image, right_image = self.extract_stereo_pair(frame_number, image_data)

        # Convert to HSV and threshold to isolate tennis ball yellow green pixels
        hsv_lower = np.array([25, 80, 80],   dtype=np.uint8)
        hsv_upper = np.array([65, 255, 255], dtype=np.uint8)
        left_mask  = cv2.inRange(cv2.cvtColor(left_image,  cv2.COLOR_RGB2HSV), hsv_lower, hsv_upper)
        right_mask = cv2.inRange(cv2.cvtColor(right_image, cv2.COLOR_RGB2HSV), hsv_lower, hsv_upper)

        # find the most circular contour in the mask and return its pixel centroid
        def get_centroid(mask):
            # Closing fills small gaps in the ball mask before contour extraction
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
            mask   = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

            # Contours give connected candidate blobs in the thresholded mask
            contour_result = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            contours = contour_result[-2]
            best, best_score = None, 0.0
            for c in contours:
                area = cv2.contourArea(c)
                perimeter = cv2.arcLength(c, True)
                if area < 50 or perimeter == 0:       # raised from 20 to 50
                    continue

                # Circularity filters out irregular blobs that are unlikely to be the ball
                circularity = (4.0 * np.pi * area) / (perimeter ** 2)
                if circularity > best_score:
                    best, best_score = c, circularity
            if best is None or best_score < 0.4:      # lowered from 0.6 to 0.4
                return None
            m = cv2.moments(best)
            if m["m00"] == 0:
                return None
            return (m["m10"] / m["m00"], m["m01"] / m["m00"])

        left_centroid  = get_centroid(left_mask)
        right_centroid = get_centroid(right_mask)

        # Ball not detected in one or both frames
        if left_centroid is None or right_centroid is None:
            return {"frame_number": frame_num, "x": 0.0, "y": 0.0, "z": 0.0, "reasonable": False, "likely_out": False}

        # Compute real world x/y/z
        u_left, v_left = left_centroid
        u_right        = right_centroid[0]
        disparity      = u_left - u_right

        if disparity <= 1.0:
            return {"frame_number": frame_num, "x": 0.0, "y": 0.0, "z": 0.0, "reasonable": False, "likely_out": False}

        # Stereo x/y come from the left image centroid, z comes from disparity
        z = (FOCAL_PX * BASELINE) / disparity
        x = (u_left - CX) * z / FOCAL_PX
        y = (v_left - CY) * z / FOCAL_PX

        return {
            "frame_number": frame_num,
            "x": x,
            "y": y,
            "z": z,
            "reasonable": True,
            "likely_out": False
        }

    # Switch the SoC into Matlab driven slave mode
    def enter_slave_mode(self):
        self.mode = MODE_SLAVE
        self.waiting_for_image = False
        self.send_slave_mode_ready()

    # Restore normal master mode operation after replay mode ends
    def reset_to_master_mode(self):
        self.mode = MODE_MASTER
        self.capture_enabled = True
        self.waiting_for_image = False
        self.last_good_position = None

    def perform_backtracking_procedure(self, frame_number):

        #TODO:
        """ My idea of the backtracking procedure:
            1. Start from the suspicious frame where the ball looked likely out
            2. Request earlier and/or nearby replay frames from Matlab
            3. For each returned frame, run the fallback image processing algorithm
            4. Track how the ball position changes across those frames
            5. Find the bounce frame or best contact point with the court
            6. Compare that position against the court lines
            7. Decide whether the ball was in or out
            8. Send the final in/out call back to Matlab
        Note: A lot of prerequisites are needed for this function to actually be implemented
        """

        return {
            "frame_number": frame_number,
            "confirmed_out": True
        }

    # Process one inbound image using the mode appropriate algorithm path
    def handle_process_image(self, frame_number, image_data):
        self.waiting_for_image = False
        self.latest_frame.put(frame_number, image_data)

        if self.mode == MODE_MASTER:
            print("Processing in MASTER mode")
            use_fpga_path = not self.disable_fpga_fast_path or frame_number == 0
            if use_fpga_path:
                result = self.fast_process_image(frame_number, image_data)
                result["used_fallback"] = False
                if not result.get("reasonable", True):
                    result = self.fallback_process_image(frame_number, image_data)
                    result["used_fallback"] = True
            else:
                result = self.fallback_process_image(frame_number, image_data)
                result["used_fallback"] = True
        else:
            print("Processing in SLAVE mode")
            result = self.fallback_process_image(frame_number, image_data)
            result["used_fallback"] = True

        self.log_position(
            result["frame_number"],
            result["x"],
            result["y"],
            result["z"]
        )

        if self.mode == MODE_MASTER and result.get("likely_out", False):
            confirmation = self.perform_backtracking_procedure(result["frame_number"])
            confirmed_out = confirmation.get("confirmed_out", True)
            result["confirmed_out"] = confirmed_out
            self.send_in_out_call(not confirmed_out)
            self.stop_capture()

        return result

    # Route one incoming command array to the appropriate SoC action
    def handle_incoming_command(self, cmd_array):
        if cmd_array is None or len(cmd_array) == 0:
            raise ValueError("cmd_array must not be empty")

        cmd = cmd_array[0]

        if cmd == CMD_RESET:
            self.reset_to_master_mode()
            return {"status": "reset_to_master"}
        
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
