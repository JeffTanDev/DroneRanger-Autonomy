#!/usr/bin/env python

import rclpy

from ros_tcp_endpoint import TcpServer


def main(args=None):
    rclpy.init(args=args)
    tcp_server = TcpServer("UnityEndpoint")
    try:
        tcp_server.start()
        tcp_server.setup_executor()
    finally:
        # Defensive cleanup: tolerate partial init / already-shutdown contexts.
        try:
            tcp_server.destroy_nodes()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
