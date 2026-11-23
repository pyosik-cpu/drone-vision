#!/usr/bin/env python3
"""逐行显示并校验SG2002发送的五个串口坐标值。"""

import argparse
import os
import select
import sys
import termios
import tty


def configure_serial(fd: int, baud: int) -> None:
    speed_name = f"B{baud}"
    if not hasattr(termios, speed_name):
        raise ValueError(f"系统不支持波特率: {baud}")
    tty.setraw(fd)
    attributes = termios.tcgetattr(fd)
    attributes[0] &= ~(termios.IXON | termios.IXOFF | termios.IXANY)
    attributes[2] |= termios.CLOCAL | termios.CREAD
    attributes[2] &= ~(termios.PARENB | termios.CSTOPB | termios.CSIZE)
    if hasattr(termios, "CRTSCTS"):
        attributes[2] &= ~termios.CRTSCTS
    attributes[2] |= termios.CS8
    speed = getattr(termios, speed_name)
    attributes[4] = speed
    attributes[5] = speed
    termios.tcsetattr(fd, termios.TCSANOW, attributes)
    termios.tcflush(fd, termios.TCIFLUSH)


def receive(device: str, baud: int) -> None:
    fd = os.open(device, os.O_RDONLY | os.O_NOCTTY | os.O_NONBLOCK)
    try:
        configure_serial(fd, baud)
        print(f"正在监听 {device} @ {baud}，按Ctrl+C退出。")
        buffer = bytearray()
        while True:
            readable, _, _ = select.select([fd], [], [], 1.0)
            if not readable:
                continue
            try:
                chunk = os.read(fd, 4096)
            except BlockingIOError:
                # CDC ACM偶尔会在select报告可读后立即返回EAGAIN。
                continue
            if not chunk:
                continue
            buffer.extend(chunk)
            while b"\n" in buffer:
                raw_line, _, remainder = buffer.partition(b"\n")
                buffer = bytearray(remainder)
                line = raw_line.decode("ascii", errors="replace").strip()
                fields = line.split(",")
                if len(fields) != 5:
                    print(f"无效行: {line}", file=sys.stderr)
                    continue
                try:
                    x_m, y_m, z_m, center_x, center_y = map(float, fields)
                except ValueError:
                    print(f"非数字行: {line}", file=sys.stderr)
                    continue
                print(
                    f"x={x_m:.4f} y={y_m:.4f} z={z_m:.4f} "
                    f"center=({center_x:.2f},{center_y:.2f})",
                    flush=True,
                )
    finally:
        os.close(fd)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("device", help="例如/dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=115200)
    arguments = parser.parse_args()
    try:
        receive(arguments.device, arguments.baud)
    except KeyboardInterrupt:
        print("\n接收结束。")
    except (OSError, ValueError) as error:
        print(f"串口接收失败: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
