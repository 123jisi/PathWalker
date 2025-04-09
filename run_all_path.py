#!/usr/bin/env python3
import subprocess
import time

def run_path(name):
    print(f"\nRunning {name} path...")
    subprocess.run(["ros2", "run", "leg_test_pkg", name])


def run_stop():
    print("\nStopping simulation...")
    subprocess.run(["ros2", "run", "leg_test_pkg", "stop"])
    time.sleep(1)


def run_reset():
    print("\nResetting simulation...")
    subprocess.run(["ros2", "run", "leg_test_pkg", "reset"])
    time.sleep(2)


def main():
    run_path("arc")
    time.sleep(1)
    run_reset()

    run_path("triangle")
    time.sleep(1)
    run_reset()

    run_path("square")


    print("\nAll paths completed.")

if __name__ == "__main__":
    main()
