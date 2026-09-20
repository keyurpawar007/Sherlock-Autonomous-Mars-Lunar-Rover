#!/usr/bin/env python3

"""
Hardware Diagnostics & Connectivity Checker.
Tests physical serial connection to STM32, camera device nodes, and sensor availability.
"""

import sys
import os
import time

try:
    import serial
except ImportError:
    serial = None


def print_header(title):
    print("\n" + "="*70)
    print(f"   {title}")
    print("="*70)


def check_serial_port(port_name='/dev/ttyUSB0', baudrate=115200):
    print_header(f"DIAGNOSTIC 1: STM32 Serial Port Check ({port_name})")

    if serial is None:
        print("  [WARN] 'pyserial' package not installed in current Python env.")
        print("  Run 'pip install pyserial' to enable serial communication.")
        return False

    if not os.path.exists(port_name):
        print(f"  [WARN] Serial device port '{port_name}' not found on filesystem.")
        print("  Available serial devices or USB ports need to be plugged in.")
        return False

    try:
        conn = serial.Serial(port_name, baudrate, timeout=0.1)
        if conn.is_open:
            print(f"  [SUCCESS] Successfully connected to STM32 on {port_name} @ {baudrate} baud!")
            conn.close()
            return True
    except Exception as e:
        print(f"  [ERROR] Failed to open serial port {port_name}: {e}")
        return False


def check_camera_device():
    print_header("DIAGNOSTIC 2: RealSense / Video Device Check")

    video_devices = [f"/dev/video{i}" for i in range(10) if os.path.exists(f"/dev/video{i}")]

    if video_devices:
        print(f"  [SUCCESS] Found {len(video_devices)} camera video device(s): {', '.join(video_devices)}")
        return True
    else:
        print("  [INFO] No USB/CSI video device nodes found on filesystem.")
        return False


def check_workspace_configuration():
    print_header("DIAGNOSTIC 3: Workspace Hardware Configuration Check")

    config_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'rover_bringup', 'config', 'rover_system.yaml'))
    if os.path.exists(config_path):
        print(f"  [SUCCESS] Master Hardware Configuration found at:")
        print(f"  -> {config_path}")
        return True
    else:
        print(f"  [ERROR] Configuration file missing at: {config_path}")
        return False


def main():
    print("\n" + "#"*70)
    print("#           PHYSICAL HARDWARE DIAGNOSTICS & VERIFICATION              #")
    print("#"*70)

    serial_ok = check_serial_port()
    camera_ok = check_camera_device()
    config_ok = check_workspace_configuration()

    print_header("HARDWARE DIAGNOSTIC SUMMARY")
    print(f"  1. STM32 Serial Bridge Status : {'READY' if serial_ok else 'MOCK FALLBACK ACTIVE'}")
    print(f"  2. Camera Sensor Status       : {'READY' if camera_ok else 'MOCK FALLBACK ACTIVE'}")
    print(f"  3. Master Config File Status  : {'READY' if config_ok else 'FAILED'}")
    print("\n  [SYSTEM NOTE] The software suite features automatic fail-safe fallback.")
    print("  When physical devices are plugged in, set 'use_hardware:=true' during launch.\n")


if __name__ == '__main__':
    main()
