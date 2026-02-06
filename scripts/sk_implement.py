#!/usr/bin/env python3
"""
Script to quickly start implementation of a work package using Gemini.
Usage: ./scripts/sk_implement.py <spec_num> <wp_num>
Example: ./scripts/sk_implement.py 001 01
"""

import sys
import subprocess
import argparse


def main():
    parser = argparse.ArgumentParser(description="Start WP implementation via Gemini")
    parser.add_argument("spec", help="Spec number (e.g., 001)")
    parser.add_argument("wp", help="Work package number (e.g., 01 or WP01)")
    parser.add_argument(
        "--yolo", action="store_false", dest="no_yolo", help="Disable YOLO mode"
    )

    args = parser.parse_args()

    spec_num = args.spec
    wp_num = args.wp

    # Standardize WP format
    if not wp_num.upper().startswith("WP"):
        # If it's just a number, pad it and add WP
        try:
            val = int(wp_num)
            wp_num = f"WP{val:02d}"
        except ValueError:
            wp_num = f"WP{wp_num.upper()}"
    else:
        # Just ensure it's uppercase and padded if possible
        prefix = wp_num[:2].upper()
        suffix = wp_num[2:]
        if suffix.isdigit():
            wp_num = f"{prefix}{int(suffix):02d}"
        else:
            wp_num = prefix + suffix.upper()

    # Standardize Spec format (pad to 3 digits if it's a number)
    if spec_num.isdigit() and len(spec_num) < 3:
        spec_num = spec_num.zfill(3)

    prompt = f"/spec-kitty.implement {wp_num} from {spec_num}"

    command = ["gemini"]
    if not args.no_yolo:
        command.append("--yolo")
    command.extend(["--prompt", prompt])

    print(
        f"\033[94m🚀 Starting implementation for {wp_num} in Spec {spec_num}...\033[0m"
    )
    print(f"\033[90mCommand: {' '.join(command)}\033[0m\n")

    try:
        subprocess.run(command)
    except FileNotFoundError:
        print("\033[31mError: 'gemini' command not found in PATH.\033[0m")
        sys.exit(1)
    except KeyboardInterrupt:
        print("\n\033[33mInterrupted.\033[0m")
        sys.exit(0)


if __name__ == "__main__":
    main()
