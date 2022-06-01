import argparse
import base64
from datetime import datetime
import os
import numpy as np
import socketio
import eventlet
import eventlet.wsgi
from flask import Flask

sio = socketio.Server()
app = Flask(__name__)


def send_control(steering_angle, throttle):
    sio.emit(
        "steer",
        data={
            'steering_angle': steering_angle.__str__(),
            'throttle': throttle.__str__()
        },
        skip_sid=True)

@sio.on('telemetry')
def telemetry(sid, data):
    if data:

        # TODO list 1*/
        # data parsing code should be putted here!*/
            ptsx = [float(d) for d in data["ptsx"]]
            ptsy = [float(d) for d in data["ptsy"]]
            px = float(data["x"])
            py = float(data["x"])
            psi = float(data["psi"])
            v = float(data["speed"])
            delta = float(data["steering_angle"])
            acceleration = float(data["throttle"])

          # TODO list 2*/
          # Algorithm that generating feedback control commands code should be putted here!*/
            send_control(0.5, 0.1)

        
    else:
        # NOTE: DON'T EDIT THIS.
        sio.emit('manual', data={}, skip_sid=True)


@sio.on('connect')
def connect(sid, environ):
    print("connect ", sid)
    send_control(0, 0)


def send_control(steering_angle, throttle):
    sio.emit(
        "steer",
        data={
            'steering_angle': steering_angle.__str__(),
            'throttle': throttle.__str__()
        },
        skip_sid=True)


if __name__ == '__main__':

    # wrap Flask application with engineio's middleware
    app = socketio.Middleware(sio, app)
    # deploy as an eventlet WSGI server
    eventlet.wsgi.server(eventlet.listen(('', 4567)), app)

