#!/usr/bin/env python3

"""
Interactive Parameter & PID Tuning CLI Tool.
Allows viewing, modifying, testing, and saving physical rover parameters,
PID gains, UART settings, and perception calibration into rover_system.yaml.
"""

import sys
import os

def load_yaml_config(file_path):
    params = {}
    if os.path.exists(file_path):
        with open(file_path, 'r') as f:
            lines = f.readlines()
            for line in lines:
                line = line.strip()
                if line and not line.startswith('#') and ':' in line:
                    parts = line.split(':', 1)
                    key = parts[0].strip()
                    val = parts[1].strip()
                    params[key] = val
    return params


def display_current_parameters(config_path):
    print("\n" + "="*70)
    print("   CURRENT ROVER HARDWARE & SYSTEM PARAMETERS")
    print("="*70)

    if not os.path.exists(config_path):
        print(f"[ERROR] Config file missing: {config_path}")
        return

    with open(config_path, 'r') as f:
        content = f.read()
        print(content)
    print("="*70)


def tune_parameter_interactive(config_path, param_key, new_value):
    if not os.path.exists(config_path):
        print(f"[ERROR] Cannot find config file: {config_path}")
        return False

    with open(config_path, 'r') as f:
        lines = f.readlines()

    modified = False
    new_lines = []
    for line in lines:
        if param_key + ':' in line:
            indent = line[:line.find(param_key)]
            new_lines.append(f"{indent}{param_key}: {new_value}\n")
            modified = True
        else:
            new_lines.append(line)

    if modified:
        with open(config_path, 'w') as f:
            f.writelines(new_lines)
        print(f"[SUCCESS] Updated parameter '{param_key}' to '{new_value}' in rover_system.yaml!")
        return True
    else:
        print(f"[WARN] Key '{param_key}' not found in configuration.")
        return False


def main():
    base_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'rover_bringup', 'config'))
    config_path = os.path.join(base_dir, 'rover_system.yaml')

    print("\n" + "#"*70)
    print("#         ROVER HARDWARE & PID PARAMETER TUNER UTILITY                 #")
    print("#"*70)

    display_current_parameters(config_path)

    # Example non-blocking argument check or interactive usage
    if len(sys.argv) >= 3:
        key_to_set = sys.argv[1]
        val_to_set = sys.argv[2]
        tune_parameter_interactive(config_path, key_to_set, val_to_set)
        display_current_parameters(config_path)
    else:
        print("\n[USAGE EXAMPLE]")
        print("  python param_tuner.py kp 1.5")
        print("  python param_tuner.py wheel_radius 0.130")
        print("  python param_tuner.py cam_pitch_deg 18.0\n")


if __name__ == '__main__':
    main()
