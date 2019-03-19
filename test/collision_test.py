#!/usr/bin/env python2

PKG = 'px4'

from pymavlink import mavutil

def main():

    # Start a connection listening to a UDP port
    the_connection = mavutil.mavlink_connection('udpin:localhost:14541')

    # Wait for the first heartbeat
    the_connection.wait_heartbeat()
    print("Heartbeat from system (system %u component %u)" % (the_connection.target_system, the_connection.target_system))


    while True:
        msg = the_connection.recv_match(type='HEARTBEAT',blocking=False)
        nav_mode = the_connection.messages['HEARTBEAT'].custom_mode

        if (nav_mode == 67371008):
            # Mission Flight Mode
            msg = the_connection.recv_match(type='COLLISION',blocking=False)
            if msg:
                print("The vehicle has crashed into an obstacle")



if __name__ == '__main__':
    main()
