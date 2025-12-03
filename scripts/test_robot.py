#!/usr/bin/env python3
"""
Test script for SiteBot robot model in MuJoCo.
Verifies the robot loads correctly and can move.
"""

import mujoco
import mujoco.viewer
import numpy as np
import time
from pathlib import Path


def main():
    # Load the robot model
    model_path = Path(__file__).parent.parent / "models" / "sitebot.xml"
    print(f"Loading model from: {model_path}")

    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)

    print(f"Model loaded successfully!")
    print(f"  - Bodies: {model.nbody}")
    print(f"  - Joints: {model.njnt}")
    print(f"  - Actuators: {model.nu}")
    print(f"  - Sensors: {model.nsensor}")

    # Test differential drive control
    print("\nStarting simulation with viewer...")
    print("Controls: Robot will drive forward, then turn")

    with mujoco.viewer.launch_passive(model, data) as viewer:
        start_time = time.time()

        while viewer.is_running():
            elapsed = time.time() - start_time

            # Simple motion pattern
            if elapsed < 3:
                # Drive forward
                data.ctrl[0] = 5.0  # left wheel
                data.ctrl[1] = 5.0  # right wheel
            elif elapsed < 5:
                # Turn right
                data.ctrl[0] = 5.0
                data.ctrl[1] = -5.0
            elif elapsed < 8:
                # Drive forward
                data.ctrl[0] = 5.0
                data.ctrl[1] = 5.0
            elif elapsed < 10:
                # Turn left
                data.ctrl[0] = -5.0
                data.ctrl[1] = 5.0
            else:
                # Stop
                data.ctrl[0] = 0.0
                data.ctrl[1] = 0.0

            # Step simulation
            mujoco.mj_step(model, data)

            # Get robot position from sensors
            pos = data.sensordata[0:3]  # chassis_pos

            # Print status every second
            if int(elapsed * 10) % 10 == 0:
                print(f"  Time: {elapsed:.1f}s | Position: ({pos[0]:.2f}, {pos[1]:.2f})")

            # Sync viewer
            viewer.sync()

            # Realtime delay
            time.sleep(model.opt.timestep)


if __name__ == "__main__":
    main()
