#!/usr/bin/env python3
"""Swarm demo: 4 Crazyflies take off, fly in a circle formation, then land."""

from crazyflie_py import Crazyswarm
import numpy as np


def main():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    Z = 1.0  # flight height
    num_cfs = len(allcfs.crazyflies)
    print(f'Swarm demo with {num_cfs} Crazyflies')

    # All take off together
    allcfs.takeoff(targetHeight=Z, duration=3.0)
    timeHelper.sleep(4.0)

    # Move to circle formation (radius 1m, centered at 0.5, 0.5)
    center = np.array([0.5, 0.5])
    radius = 1.0
    for i, cf in enumerate(allcfs.crazyflies):
        angle = 2 * np.pi * i / num_cfs
        x = center[0] + radius * np.cos(angle)
        y = center[1] + radius * np.sin(angle)
        cf.goTo(np.array([x, y, Z]), 0, 3.0)
    timeHelper.sleep(4.0)

    # Rotate the formation (each drone moves to the next position)
    for step in range(4):
        for i, cf in enumerate(allcfs.crazyflies):
            angle = 2 * np.pi * ((i + step + 1) % num_cfs) / num_cfs
            x = center[0] + radius * np.cos(angle)
            y = center[1] + radius * np.sin(angle)
            cf.goTo(np.array([x, y, Z]), 0, 3.0)
        timeHelper.sleep(4.0)

    # Return to original positions before landing
    for cf in allcfs.crazyflies:
        pos = np.array(cf.initialPosition) + np.array([0, 0, Z])
        cf.goTo(pos, 0, 3.0)
    timeHelper.sleep(4.0)

    # Land all
    allcfs.land(targetHeight=0.04, duration=3.0)
    timeHelper.sleep(4.0)


if __name__ == '__main__':
    main()
