# -*- coding: utf-8 -*-

from image_cache import create_dummy_image
from soc_protocol import SoCProtocol, CMD_PROCESS_IMAGE, CMD_SLAVE_MODE, CMD_RESET


# Testingg the protocol with dummy images in both SoC modes
def main():
    soc = SoCProtocol()

    dummy0 = create_dummy_image(752, 480, 120)
    print(soc.handle_incoming_command([CMD_PROCESS_IMAGE, 100, dummy0]))
    latest = soc.latest_frame.get()
    if latest is not None:
        print("Latest frame:", latest["frame_number"])

    print(soc.handle_incoming_command([CMD_SLAVE_MODE]))

    dummy1 = create_dummy_image(752, 480, 150)
    print(soc.handle_incoming_command([CMD_PROCESS_IMAGE, 101, dummy1]))

    latest = soc.latest_frame.get()
    if latest is not None:
        print("Latest frame after slave mode image:", latest["frame_number"])

    print(soc.handle_incoming_command([CMD_RESET]))


if __name__ == '__main__':
    main()
