# -*- coding: utf-8 -*-

from image_cache import LatestFrameStore


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
    def __init__(self, command_sender=None):
        self.mode = MODE_MASTER
        self.latest_frame = LatestFrameStore()
        self.capture_enabled = True
        self.command_sender = command_sender
        self.waiting_for_image = False

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
        return {
            "frame_number": frame_number,
            "x": 0.0,
            "y": 0.0,
            "z": 0.0,
            "reasonable": True,
            "likely_out": False
        }

    # Run the slower fallback processing path used for replay or bad calls
    def fallback_process_image(self, frame_number, image_data):
        return {
            "frame_number": frame_number,
            "x": 0.0,
            "y": 0.0,
            "z": 0.0,
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
