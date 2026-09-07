#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from rclpy.logging import LoggingSeverity
from sensor_msgs.msg import Range
import serial
import threading
import queue
import time

INDEX_TO_SENSOR = {
    0: 'us_top_right',
    1: 'us_top_left',	
    2: 'us_mid_right',
    3: 'us_mid_left',
    4: 'us_bot_right',
    5: 'us_bot_left',
}

SENSOR_LABELS = [INDEX_TO_SENSOR[i][len('us_'):] for i in range(len(INDEX_TO_SENSOR))]
NUM_SENSORS   = 6

class UltrasonicNode(Node):
    def __init__(self, debug: bool = False):
        super().__init__('ultrasonic_node')
        if debug:
            self.get_logger().set_level(LoggingSeverity.DEBUG)
        self.declare_parameter('serial_port', '/dev/ttyUltrasonic')
        self.declare_parameter('baud_rate', 115200)
        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('baud_rate').value

        self.pubs = {
            name: self.create_publisher(Range, f'/ultrasonic/{name}', 10)
            for name in INDEX_TO_SENSOR.values()
        }

        self.ser = self._open_serial_with_retry(port, baud, timeout=1)
        self.ser.reset_input_buffer()
        self.get_logger().info(f'Opened {port} @ {baud}')
        self._queue = queue.Queue(maxsize=20)

        # Timer 10ms chạy trong ROS executor → an toàn publish/log
        self.create_timer(0.01, self._publish_loop)

        self._stop_event = threading.Event()
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()
        self.get_logger().info('Read thread started')

    def _open_serial_with_retry(self, port: str, baud: int, timeout: float,
                                 retries: int = 4, delay: float = 0.6) -> serial.Serial:
        last_err = None
        for attempt in range(1, retries + 1):
            try:
                return serial.Serial(port, baud, timeout=timeout)
            except (serial.SerialException, OSError) as e:
                last_err = e
                self.get_logger().warn(
                    f'Serial open failed on {port} (attempt {attempt}/{retries}): {e}'
                )
                time.sleep(delay)
        raise last_err

    # ------------------------------------------------------------------ #
    #  Serial thread — chỉ đọc và đẩy vào queue, KHÔNG gọi ROS API       #
    # ------------------------------------------------------------------ #
    def _read_loop(self):
        while not self._stop_event.is_set():
            try:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if not line:
                    continue

                try:
                    self._queue.put_nowait(line)
                except queue.Full:
                    # Xóa dòng cũ nhất, đẩy dòng mới vào
                    try:
                        self._queue.get_nowait()
                    except queue.Empty:
                        pass
                    self._queue.put_nowait(line)

            except serial.SerialException as e:
                print(f'[ultrasonic] SerialException: {e}')
                break
            except Exception as e:
                print(f'[ultrasonic] Unexpected: {type(e).__name__}: {e}')

    # ------------------------------------------------------------------ #
    #  Timer callback — chạy trong ROS executor, an toàn gọi ROS API     #
    # ------------------------------------------------------------------ #
    def _publish_loop(self):
        # Xử lý hết queue trong 1 lần callback
        while not self._queue.empty():
            try:
                line = self._queue.get_nowait()
                self._process_line(line)
            except queue.Empty:
                break

    # ------------------------------------------------------------------ #
    #  Frame parser                                                      #
    # ------------------------------------------------------------------ #
    def _process_line(self, line: str):
        if not line.startswith('$'):
            self.get_logger().warn(f'Bad start: {repr(line)}')
            return
        if '*' not in line:
            self.get_logger().warn(f'No checksum: {repr(line)}')
            return

        try:
            body, chk_str = line[1:].rsplit('*', 1)

            vals = [int(x) for x in body.split(',')]
            if len(vals) != NUM_SENSORS:
                self.get_logger().warn(f'Wrong field count: {len(vals)} | {line}')
                return

            chk_calc = 0
            for v in vals:
                chk_calc ^= (v & 0xFF)

            chk_recv = int(chk_str)
            if chk_calc != chk_recv:
                self.get_logger().warn(
                    f'Checksum FAIL: calc={chk_calc} recv={chk_recv} | {line}'
                )
                return

            stamp = self.get_clock().now().to_msg()
            for idx, dist_cm in enumerate(vals):
                name = INDEX_TO_SENSOR[idx]
                self.pubs[name].publish(self._make_range(name, dist_cm, stamp))

            self.get_logger().debug('  '.join(f'{SENSOR_LABELS[i]}={vals[i]}' for i in range(NUM_SENSORS)))


        except ValueError as e:
            self.get_logger().warn(f'ValueError: {e} | {repr(line)}')
        except IndexError as e:
            self.get_logger().warn(f'IndexError: {e} | {repr(line)}')

    # ------------------------------------------------------------------ #
    #  Message builder                                                   #
    # ------------------------------------------------------------------ #
    def _make_range(self, frame_id: str, dist_cm: int, stamp) -> Range:
        msg = Range()
        msg.header.stamp    = stamp
        msg.header.frame_id = frame_id
        msg.radiation_type  = Range.ULTRASOUND
        msg.field_of_view   = 0.26
        msg.min_range       = 0.01
        msg.max_range       = 2.5
        msg.range           = float('inf') if dist_cm >= 200 else dist_cm / 100.0
        return msg

    # ------------------------------------------------------------------ #
    #  Cleanup                                                             #
    # ------------------------------------------------------------------ #
    def destroy_node(self):
        self.get_logger().info('Shutting down...')
        self._stop_event.set()
        self._thread.join(timeout=2.0)
        if self.ser.is_open:
            self.ser.close()
            self.get_logger().info('Serial port closed')
        super().destroy_node()


def main(args=None):
    argv = sys.argv if args is None else args
    debug = '--debug' in argv
    argv = [a for a in argv if a != '--debug']

    rclpy.init(args=argv)
    node = UltrasonicNode(debug=debug)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()