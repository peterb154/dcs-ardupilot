'''
Test loop for connecting a udp physics engine to ardupilot
See the following for information:
https://github.com/ArduPilot/ardupilot/tree/master/libraries/SITL/examples/JSON
'''
import asyncio
import struct
import json
import socket
import time
import logging


class EchoServerProtocol:
    def __init__(self):
        self.last_frame = -1

    def connection_made(self, transport):
        self.transport = transport

    def datagram_received(self, data, addr):
        # will be invoked for each udp packet received
        # input will be binary encoded input with the following format
        parse_format = 'HHI16H'
        decoded = struct.unpack(parse_format, data)
        input = {
            "magic": decoded[0], # a constant of 18458
            "frame_rate_hz": decoded[1], # can be ignored by the physics backend. This measures how fast updates are happening
            "frame_count": decoded[2], # increments for each output frame sent by ArduPilot. Used to detect dropped frames
            "pwm": decoded[3:] # 16 pwn values range 1000-2000
        }
        # Create a response object
        response = {
            "timestamp": time.time(), # (s) physics time
            "imu": {
                "gyro": [0,0,0], # (roll, pitch, yaw) (radians/sec) body frame
                "accel_body": [0,0,0], # (x, y, z) (m/s^2) body frame
            },
            "position": [0,0,0], # (north, east, down) (m) earth frame
            "attitude": [0,0,0], # (roll, pitch, yaw) (radians)
            "velocity": [0,0,0], # (north, east, down) (m/s) earth frame
#             "airspeed": 0, # (m/s) optional
#             "windvane": { # optional apparent wing
#                 "direction": 0, # (radians) clockwise relative to the front, i.e. 0 = head to wind
#                 "speed": 0, # (m/s)
#             },
#             "rng_1": 0 # (m) optional range finder - AGL??
        }
        # response must be a json string, with no spaces, prefixed & suffixed with a '\n' ascii encoded binary
        json_response = f"\n{json.dumps(response, separators=(',', ':'))}\n".encode("ascii")
        # Send it back to the host and port we recieved the datagram
        self.transport.sendto(json_response, addr)

        # print every 5000th frame so we can see something is happening
        if input["frame_count"] % 5000 == 0:
            logging.info(f"rcvd: {json.dumps(input)}")
            logging.info(f"xmit {addr} : {json_response}")

        # let user know if frames were dropped
        if input["frame_count"] - 1 != self.last_frame:
            dropped_frames = input["frame_count"] - self.last_frame
            logging.warning(f"dropped {dropped_frames} frames" )
        self.last_frame = input["frame_count"]


logging.getLogger().setLevel("INFO")
loop = asyncio.get_event_loop()
host = "0.0.0.0"
port = 9002
logging.info(f"Listening on udp socket - host:{host} port:{port}")

# One protocol instance will be created to serve all client requests
listen = loop.create_datagram_endpoint(EchoServerProtocol, local_addr=(host, port))
transport, protocol = loop.run_until_complete(listen)

try:
    loop.run_forever()
except KeyboardInterrupt:
    pass
finally:
    transport.close()
    loop.close()
