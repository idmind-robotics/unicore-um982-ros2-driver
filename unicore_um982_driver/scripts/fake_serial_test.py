#!/usr/bin/env python3
"""Headless smoke test for the unicore_um982_driver using a local pseudo-terminal.

No GPS hardware is required. It verifies the two halves of the new architecture:

  1. Corrections bridge: the node connects to a local TCP server (standing in for
     str2str's tcpsvr output), receives RTCM bytes and forwards them into the
     serial port. We read them back on the PTY master side.
  2. Fix parsing: we write a fake PVTSLN line into the PTY master side (as the
     receiver would) and check that the node publishes sensor_msgs/NavSatFix on
     /gps/fix.

Usage:
    python3 scripts/fake_serial_test.py [--workspace ~/ros2_ws] [--port 40001]
"""

import argparse
import os
import pty
import random
import select
import shutil
import signal
import socket
import subprocess
import sys
import tempfile
import threading
import time
import yaml

RTCM_MARKER = bytes([0xD3, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05])
PVTSLN = (
    "PVTSLNA,84,GPS,FINE,2293,376607.200,0,0,0;NARROW_INT,62.604,38.805979,9.234847,"
    "0.280,0.370,0.410,1.6,NARROW_INT,62.604,38.805979,9.234847,59.171,"
    "16,16,16,16,0.052,0.044,0.068,NONE,0.000,0.000,0.000\r\n"
)


class FakeCorrectionsServer:
    """Minimal TCP server standing in for `str2str -out tcpsvr://127.0.0.1:port`."""

    def __init__(self, port):
        self.srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.srv.bind(("127.0.0.1", port))
        self.srv.listen(1)
        self.conn = None
        self.stop = threading.Event()

    def run(self):
        while not self.stop.is_set():
            try:
                self.srv.settimeout(0.5)
                conn, _ = self.srv.accept()
            except socket.timeout:
                continue
            with conn:
                self.conn = conn
                try:
                    while not self.stop.is_set():
                        conn.settimeout(0.5)
                        try:
                            if conn.send(RTCM_MARKER) == 0:
                                break
                        except socket.timeout:
                            pass
                        except (ConnectionResetError, BrokenPipeError):
                            break
                        time.sleep(1.0)
                finally:
                    self.conn = None
        self.srv.close()

    def start(self):
        threading.Thread(target=self.run, daemon=True).start()

    def close(self):
        self.stop.set()
        if self.conn:
            try:
                self.conn.close()
            except OSError:
                pass
        self.srv.close()


def wait_for(condition, timeout, step=0.1):
    deadline = time.time() + timeout
    while time.time() < deadline:
        if condition():
            return True
        time.sleep(step)
    return False


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workspace", default=os.path.expanduser("~/ros2_ws"),
                        help="ROS 2 workspace root (default: ~/ros2_ws)")
    parser.add_argument("--port", type=int, default=40001,
                        help="Local corrections TCP port (default: 40001)")
    args = parser.parse_args()

    if shutil.which("ros2") is None:
        sys.exit("ERROR: 'ros2' not found. Source your ROS 2 environment first.")

    install_dir = os.path.join(args.workspace, "install")
    if not os.path.isdir(install_dir):
        sys.exit(f"ERROR: workspace install dir not found: {install_dir}")

    setup = (f"source /opt/ros/humble/setup.bash && "
             f"source {install_dir}/setup.bash && ")

    master_fd, slave_fd = pty.openpty()
    slave_path = os.ttyname(slave_fd)
    tmp_dir = tempfile.mkdtemp(prefix="unicore_um982_test_")
    node_log = open(os.path.join(tmp_dir, "node.log"), "w")
    # Hermetic: a unique port per run so leftover processes from previous runs
    # cannot steal the corrections connection.
    bridge = FakeCorrectionsServer(random.randint(41000, 44000))
    local_port = bridge.srv.getsockname()[1]
    node = None
    rtcm_seen = [False]
    fix_ok = [False]

    try:
        # Minimal config fully overriding YAML defaults, pointing at the PTY.
        cfg = {
            "unicore_um982_driver": {
                "ros__parameters": {
                    "port": slave_path,
                    "baudrate": 115200,
                    "config_commands": ["CONFIG COM3 115200"],
                    "ntrip_server": "unused",
                    "ntrip_port": 2101,
                    "ntrip_mountpoint": "unused",
                    "ntrip_user": "user",
                    "ntrip_pass": "password",
                    "ntrip_local_port": local_port,
                    "enable_ntrip": True,
                    "frame_id": "gps_link",
                    "heading_offset_deg": 0.0,
                    "heading_stddev_deg": 0.5,
                }
            }
        }
        cfg_path = os.path.join(tmp_dir, "config.yaml")
        with open(cfg_path, "w") as f:
            yaml.safe_dump(cfg, f)

        bridge.start()

        cmd = (f"{setup} ros2 run unicore_um982_driver unicore_um982_driver_node "
               f"--ros-args --log-level DEBUG --params-file {cfg_path}")
        # Start the node in its own session so we can kill the whole process
        # group; bash -c otherwise orphans the ros2 child on terminate().
        node = subprocess.Popen(cmd, shell=True, executable='/bin/bash',
                                start_new_session=True,
                                stdout=node_log, stderr=subprocess.STDOUT)

        # Read whatever the node forwards into the serial port (master side).
        def reader():
            while True:
                try:
                    r, _, _ = select.select([master_fd], [], [], 0.2)
                    if not r:
                        continue
                    data = os.read(master_fd, 4096)
                    if not data:
                        return
                    if RTCM_MARKER in data:
                        rtcm_seen[0] = True
                except OSError as e:
                    print(f"[reader] {e}", flush=True)
                    return
        threading.Thread(target=reader, daemon=True).start()

        # Feed fake receiver output (PVTSLN) so the node has data to publish. Wait for
# the node to open and flush the pty first (fresh-PTY behaviour would otherwise
# discard bytes queued before the slave is configured).
        def writer():
            time.sleep(1.5)
            count = 0
            while True:
                try:
                    os.write(master_fd, PVTSLN.encode())
                    count += 1
                except OSError:
                    return
                time.sleep(0.2)
        threading.Thread(target=writer, daemon=True).start()

        # Watch for the published fix via `ros2 topic echo`. Explicit QoS skips the
        # daemon-backed publisher query (which breaks when a stale daemon exists).
        # Retried: a fresh subscriber occasionally misses the discovery handshake.
        for _ in range(3):
            echo_cmd = (f"{setup} timeout 15 ros2 topic echo /gps/fix --once "
                        f"--qos-reliability reliable --qos-durability volatile")
            echo = subprocess.run(echo_cmd, shell=True, executable='/bin/bash',
                                  stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                  text=True, timeout=20)
            if "latitude:" in echo.stdout:
                break
            time.sleep(1)
        fix_ok[0] = "latitude:" in echo.stdout

        if not wait_for(lambda: rtcm_seen[0], 5.0):
            print("FAIL: corrections sent by the TCP bridge never reached the serial port")
        elif not fix_ok[0]:
            print("FAIL: /gps/fix not published (or bad contents) for the fake PVTSLN line")
        else:
            print("PASS: bridge -> serial forwarding works, parser publishes /gps/fix")
            print(f"      (node serial device: {slave_path}, corrections port: {local_port})")

        print(f"      [rtcm_seen={rtcm_seen[0]} fix_ok={fix_ok[0]}]")

    finally:
        node_log.flush()
        if not (rtcm_seen[0] and fix_ok[0]):
            print(f"--- node.log ({tmp_dir}/node.log) ---")
            with open(os.path.join(tmp_dir, "node.log")) as f:
                print(f.read())
        if node:
            try:
                os.killpg(node.pid, signal.SIGTERM)
                node.wait(timeout=5)
            except (ProcessLookupError, subprocess.TimeoutExpired):
                try:
                    os.killpg(node.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
        bridge.close()
        node_log.close()
        os.close(master_fd)
        os.close(slave_fd)
        shutil.rmtree(tmp_dir, ignore_errors=True)

    return 0 if (rtcm_seen[0] and fix_ok[0]) else 1


if __name__ == "__main__":
    sys.exit(main())