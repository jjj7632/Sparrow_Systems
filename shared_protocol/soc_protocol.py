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


class SoCProtocol(object):
    # Initialize SoC mode, the capture state, and the outbound commands
    def __init__(self, command_sender=None, fpga_cache=None):
        self.mode = MODE_MASTER
        self.latest_frame = LatestFrameStore()
        self.capture_enabled = True
        self.command_sender = command_sender
        self.waiting_for_image = False
        self.fpga_cache = fpga_cache

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

    # Signal that the SoC is ready for slave-mode replay commands
    def send_slave_mode_ready(self):
        self.send_command([CMD_SLAVE_MODE_READY])

    # Run the fast primary processing path used in master mode
    def fast_process_image(self, frame_number, image_data):
        # TODO:
        """
        # 1. Send the image info to the FPGA fast path
                - this requires setting up DDR, probably utilizing AXI-Lite for actual image caching
                  so that the fpga can read (Josh is working on this as his next task)
                        a.) allocate a 2 DDR frame ping pong buffer that Python can fille and FPGA can read from
                        b.) copy the latest image into it
                        c.) store its physical address
                        d.) write that address to FPGA registers
                        e.) trigger the FPGA
        # 2. Read back the FPGA estimated ball position
        # 3. Convert the FPGA output into x/y/z values in Python
        # 4. Check whether the FPGA result is reasonable compared to recent history
        # 5. Flag whether the result should be sent to the fallback path
        # 6. Flag whether this frame looks like a possible out call
        # 7. Return the frame number, x/y/z position, reasonableness, and likely_out status
        """

        if self.fpga_cache is None:
            raise RuntimeError("fpga_cache must be configured with a DMA backed PingPongFpgaCache")

        self.fpga_cache.submit_frame(frame_number=frame_number, image_data=image_data)
        result = self.fpga_cache.read_result()

        print(result)
        
        
        # Reasonable Logic... blah blah blah
        reasonable = True

        # Likely out Logic... blah blah blah
        likely_out = False

        return {
            "frame_number": frame_number,
            "x": result["x"],
            "y": result["y"],
            "z": result["z"],
            "reasonable": reasonable,
            "likely_out": likely_out
        }

    # Run the slower fallback processing path used for replay or bad calls
    def extract_stereo_pair(self, frame_number, image_data):
        if isinstance(image_data, dict) and "left_image" in image_data and "right_image" in image_data:
            return frame_number, image_data["left_image"], image_data["right_image"]

        raise ValueError("fallback_process_image expects an already-paired stereo frame")

    # Run the slower fallback processing path used for replay or bad calls
    def fallback_process_image(self, frame_number, image_data):
        # Camera constants
        FOCAL_PX = 10.0 / 0.006
        CX       = 1920 / 2.0
        CY       = 1080 / 2.0
        BASELINE = 0.10

        frame_num, left_image, right_image = self.extract_stereo_pair(frame_number, image_data)

        # Convert to HSV and threshold to isolate tennis ball yellow-green pixels
        hsv_lower = np.array([25, 80, 80],   dtype=np.uint8)
        hsv_upper = np.array([65, 255, 255], dtype=np.uint8)
        left_mask  = cv2.inRange(cv2.cvtColor(left_image,  cv2.COLOR_RGB2HSV), hsv_lower, hsv_upper)
        right_mask = cv2.inRange(cv2.cvtColor(right_image, cv2.COLOR_RGB2HSV), hsv_lower, hsv_upper)

        # Find the most circular contour in the mask and return its pixel centroid
        def get_centroid(mask):
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
            mask   = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

            contour_result = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            contours = contour_result[-2]
            best, best_score = None, 0.0
            for c in contours:
                area = cv2.contourArea(c)
                perimeter = cv2.arcLength(c, True)
                if area < 50 or perimeter == 0:       # raised from 20 to 50
                    continue
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

    # Process one inbound image using the mode-appropriate algorithm path
    def handle_process_image(self, frame_number, image_data):
        self.waiting_for_image = False
        self.latest_frame.put(frame_number, image_data)

        if self.mode == MODE_MASTER:
            print("Processing in MASTER mode")
            result = self.fast_process_image(frame_number, image_data)
            result["used_fallback"] = False
            if not result.get("reasonable", True):
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
