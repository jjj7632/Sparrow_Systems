#!/usr/bin/env python3

import socket
import struct

import numpy as np


class NumpySocket(object):
    # Set up the socket helper with fixed image settings for this connection
    def __init__(self, image_shape=(1080, 1920, 3), image_dtype=np.uint8):
        self.address = 0
        self.port = 0
        self.client_connection = self.client_address = None
        self.image_shape = tuple(image_shape)
        self.image_dtype = np.dtype(image_dtype)
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    # Clean up sockets when the helper object is destroyed
    def __del__(self):
        try:
            self.close()
        except OSError:
            pass

    # Start listening for a client on the given TCP port
    def startServer(self, port, address=''):
        self.address = address
        self.port = port

        self.socket.bind((self.address, self.port))
        self.socket.listen(1)
        self.client_connection, self.client_address = self.socket.accept()

    # Connect to a remote TCP server
    def startClient(self, address, port):
        self.address = address
        self.port = port
        self.socket.connect((self.address, self.port))

    start_server = startServer
    start_client = startClient

    # Close the accepted client socket and the base socket
    def close(self):
        try:
            self.client_connection.close()
        except AttributeError:
            pass
        except OSError:
            pass
        self.client_connection = self.client_address = None

        if self.socket is not None:
            try:
                self.socket.close()
            except OSError:
                pass
            self.socket = None

    # Return the active socket object for send and receive calls
    def activeSocket(self):
        if self.client_connection is not None:
            return self.client_connection
        if self.socket is None:
            raise ConnectionError("Socket is closed")
        return self.socket

    # Return the byte count for one image using the configured shape and dtype
    def imageByteCount(self):
        return int(np.prod(self.image_shape, dtype=np.int64)) * self.image_dtype.itemsize

    # Read an exact number of bytes from the active socket
    def recvExact(self, size):
        buffer_data = bytearray()
        socket_obj = self.activeSocket()

        while len(buffer_data) < size:
            data = socket_obj.recv(size - len(buffer_data))
            if not data:
                if len(buffer_data) == 0:
                    return None
                raise ConnectionError("Connection closed while receiving data")
            buffer_data.extend(data)

        return bytes(buffer_data)

    # Send a NumPy image buffer as raw bytes
    def send(self, frame):
        if not isinstance(frame, np.ndarray):
            raise TypeError("input frame is not a valid numpy array")

        frame = np.ascontiguousarray(frame, dtype=self.image_dtype)
        if frame.size != int(np.prod(self.image_shape, dtype=np.int64)):
            raise ValueError("input frame size does not match configured image_shape")
        socket_obj = self.activeSocket()
        socket_obj.sendall(frame.tobytes(order="C"))

    # Receive a fixed size image buffer and reshape it to the configured size
    def receive(self):
        socket_obj = self.activeSocket()
        if socket_obj is None:
            raise ConnectionError("Socket is closed")

        frame_buffer = self.recvExact(self.imageByteCount())
        if frame_buffer is None:
            return None

        frame = np.frombuffer(frame_buffer, dtype=self.image_dtype)
        return frame.reshape(self.image_shape)

    # Send a one byte command value
    def sendCmd(self, cmd):
        self.activeSocket().sendall(struct.pack("!B", int(cmd)))

    # Receive a one-byte command value
    def receiveCmd(self):
        data = self.recvExact(1)
        if data is None:
            return None
        return struct.unpack("!B", data)[0]

    # Send a signed 32-bit integer value
    def sendInt32(self, value):
        self.activeSocket().sendall(struct.pack("!i", int(value)))

    # Receive a signed 32-bit integer value
    def receiveInt32(self):
        data = self.recvExact(4)
        if data is None:
            return None
        return struct.unpack("!i", data)[0]

    # Send an unsigned 8-bit integer value
    def sendUint8(self, value):
        self.activeSocket().sendall(struct.pack("!B", int(value)))

    # Receive an unsigned 8-bit integer value
    def receiveUint8(self):
        data = self.recvExact(1)
        if data is None:
            return None
        return struct.unpack("!B", data)[0]

    # Send a 32 bit floating-point value
    def sendFloat32(self, value):
        self.activeSocket().sendall(struct.pack("!f", float(value)))

    # Receive a 32 bit floating-point value
    def receiveFloat32(self):
        data = self.recvExact(4)
        if data is None:
            return None
        return struct.unpack("!f", data)[0]
