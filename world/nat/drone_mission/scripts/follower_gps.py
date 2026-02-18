#!/usr/bin/env python3
from pymavlink import mavutil
import time

LEADER_PORT = 14550
OFFSET_LAT = -5e-7   # ~5 m detrás
OFFSET_LON = 0
DT = 0.5


def main():
    leader = mavutil.mavlink_connection(f'udp:127.0.0.1:{LEADER_PORT}')
    leader.wait_heartbeat()
    print("[OK] follower conectado al leader")

    while True:
        msg = leader.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        if not msg or msg.lat == 0:
            continue

        lat = msg.lat
        lon = msg.lon
        alt = msg.relative_alt / 1000.0

        target_lat = lat + int(OFFSET_LAT * 1e7)
        target_lon = lon + int(OFFSET_LON * 1e7)

        leader.mav.set_position_target_global_int_send(
            int(time.time() * 1000) & 0xFFFFFFFF,
            leader.target_system,
            leader.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            0b110111111000,
            target_lat,
            target_lon,
            alt,
            0, 0, 0,
            0, 0, 0,
            0, 0
        )

        time.sleep(DT)


if __name__ == "__main__":
    main()

