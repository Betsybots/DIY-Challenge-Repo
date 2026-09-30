#!/usr/bin/env python3
"""Dummy competition start trigger: publishes std_msgs/Bool data=true on
/green_light so waypoint_sequencer (wait_for_green_light:=true) starts
sending waypoints.

waypoint_sequencer subscribes with default (volatile) QoS and starts on the
first data=true it receives, so a single message sent before it subscribes is
lost. This script therefore keeps publishing at --rate until Ctrl-C (repeats
are harmless, the sequencer starts only once). With --once it waits until at
least one subscriber is connected, publishes a few times, and exits.

Usage:
  python3 green_light_publisher.py                 # true at 2 Hz until Ctrl-C
  python3 green_light_publisher.py --delay 5       # 5 s countdown first
  python3 green_light_publisher.py --once          # wait for sequencer, send, exit
Equivalent one-liner:
  ros2 topic pub -r 2 /green_light std_msgs/msg/Bool "{data: true}"
"""
import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--topic', default='/green_light')
    ap.add_argument('--rate', type=float, default=2.0, help='publish rate in Hz (default 2)')
    ap.add_argument('--delay', type=float, default=0.0, help='seconds to wait before going green')
    ap.add_argument('--once', action='store_true',
                    help='wait for a subscriber, publish true a few times, then exit')
    ap.add_argument('--wait-timeout', type=float, default=60.0,
                    help='--once: give up if no subscriber appears within this many seconds')
    args, ros_args = ap.parse_known_args()
    if args.rate <= 0.0:
        sys.exit('--rate must be > 0')

    rclpy.init(args=ros_args)
    node = Node('dummy_green_light')
    pub = node.create_publisher(Bool, args.topic, 10)
    msg = Bool(data=True)
    log = node.get_logger()

    try:
        if args.delay > 0.0:
            log.info(f'Going green on {args.topic} in {args.delay:.1f} s ...')
            end = time.monotonic() + args.delay
            while rclpy.ok() and time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=0.1)

        if args.once:
            log.info(f'Waiting for a subscriber on {args.topic} ...')
            end = time.monotonic() + args.wait_timeout
            while rclpy.ok() and pub.get_subscription_count() == 0:
                if time.monotonic() > end:
                    log.error(f'No subscriber on {args.topic} after {args.wait_timeout:.0f} s — '
                              'is waypoint_sequencer running with wait_for_green_light:=true?')
                    return 1
                rclpy.spin_once(node, timeout_sec=0.1)
            for _ in range(3):
                pub.publish(msg)
                rclpy.spin_once(node, timeout_sec=0.1)
            log.info(f'GREEN LIGHT sent on {args.topic} ({pub.get_subscription_count()} subscriber(s)).')
            return 0

        log.info(f'Publishing GREEN LIGHT (data: true) on {args.topic} at {args.rate:g} Hz — Ctrl-C to stop.')
        node.create_timer(1.0 / args.rate, lambda: pub.publish(msg))
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
