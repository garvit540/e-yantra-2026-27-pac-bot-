"""Boilerplate for PB Task 1B.

Subscribes to the simulator's sensor topic, logs each reading, and publishes
a wheel velocity command back. Fill in your control logic where marked.

Run (three terminals):
    mosquitto
    ./task_1b_launch
    python3 task_1b_boilerplate.py
"""
import json

import paho.mqtt.client as mqtt

MQTT_HOST = "localhost"
MQTT_PORT = 1883
TOPIC_SENSORS = "pacbot/sensors"      # simulator publishes, this file subscribes
TOPIC_WHEEL_VEL = "pacbot/wheel_vel"  # this file publishes, simulator subscribes

# ---------------- Controller tuning (edit these while testing) ----------------
BASE_VEL = 25.0        # rad/s, forward wheel speed
MAX_VEL = 30.0        # rad/s, wheel speed clamp
TURN_VEL = 4.0        # rad/s, in-place turn speed at corners
SET_DIST = 0.05       # m, desired distance from the LEFT wall
FRONT_STOP = 0.11     # m, front blocked below this -> turn away
WALL_LOST = 0.20     # m, left reading above this -> wall gap / opening
MAX_RANGE = 0.50      # m, clamp for inf / bad readings
KP = 40.0
KI = 0.05
KD = 18.0
I_LIMIT = 0.5

# PID state (persists between callbacks)
_pid = {"prev_err": 0.0, "integral": 0.0}


def _mqtt_client():
    # paho-mqtt >= 2.0 requires picking a callback API version explicitly.
    try:
        return mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    except AttributeError:
        return mqtt.Client()


def on_message(client, userdata, msg):
    data = json.loads(msg.payload.decode())

    fl = data["fl"]            # Front-left ToF distance readings
    fr = data["fr"]            # Front-right ToF distance readings
    sl = data["sl"]            # Side-left ToF distance readings 
    sr = data["sr"]            # Side-right ToF distance readings 
    yaw_rate = data["gyro"][2]  # rad/s about z
    dt = data["dt"]            # s, simulator timestep

    print(f"fl={fl:.3f} fr={fr:.3f} sl={sl:.3f} sr={sr:.3f} "
          f"yaw_rate={yaw_rate:+.3f} dt={dt:.4f}")

    # ---------------- WALL FOLLOWING + PID (left-wall follower) ----------------
    def _clean(x):
        # guard against inf / nan / negative readings
        try:
            if x != x or x < 0:
                return MAX_RANGE
            return min(float(x), MAX_RANGE)
        except (TypeError, ValueError):
            return MAX_RANGE

    d_fl, d_fr, d_sl, d_sr = _clean(fl), _clean(fr), _clean(sl), _clean(sr)
    front = min(d_sl, d_sr)
    dt_safe = dt if dt and dt > 1e-6 else 1e-3

    if front < FRONT_STOP:
        # Wall ahead: rotate in place away from the closer side.
        # Prefer turning right (keeps left wall on our side); turn left
        # only if the right side is clearly more open (dead end / U-turn).
        _pid["integral"] = 0.0
        _pid["prev_err"] = 0.0
        if d_fr >= d_fl:
            left_vel, right_vel = TURN_VEL, -TURN_VEL    # turn right
        else:
            left_vel, right_vel = -TURN_VEL, TURN_VEL    # turn left

    elif d_fl > d_fr:
        # Left wall lost (opening): curve left to re-find the wall.
        _pid["integral"] = 0.0
        _pid["prev_err"] = 0.0
        left_vel = BASE_VEL * 0.7
        right_vel = BASE_VEL

    else:
        # Normal wall following: PID on distance to left wall.
        # error > 0 -> too far from wall -> steer left
        error = d_fl - d_fr
        _pid["integral"] += error * dt_safe
        _pid["integral"] = max(-I_LIMIT, min(I_LIMIT, _pid["integral"]))
        derivative = (error - _pid["prev_err"]) / dt_safe
        _pid["prev_err"] = error

        corr = KP * error + KI * _pid["integral"] + KD * derivative
        corr = max(-BASE_VEL, min(BASE_VEL, corr))

        # Slow down a bit as we approach a wall ahead
        speed = BASE_VEL * min(1.0, max(0.4, front / 0.30))

        left_vel = speed - corr
        right_vel = speed + corr

    left_vel = max(-MAX_VEL, min(MAX_VEL, left_vel))
    right_vel = max(-MAX_VEL, min(MAX_VEL, right_vel))
    # ---------------------------------------------------------------------------

    client.publish(TOPIC_WHEEL_VEL, json.dumps({
        "left": float(left_vel), "right": float(right_vel),
    }))


def main():
    client = _mqtt_client()
    client.on_message = on_message
    client.connect(MQTT_HOST, MQTT_PORT)
    client.subscribe(TOPIC_SENSORS)
    client.loop_forever()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
