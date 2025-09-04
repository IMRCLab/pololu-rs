#!/usr/bin/env python

from crazyflie_py import Crazyswarm


def main():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    # arm (one by one)
    for cf in allcfs.crazyflies:
        cf.arm(True)
        timeHelper.sleep(1.0)

    timeHelper.sleep(2.0)

    # disarm (broadcast)
    allcfs.arm(False)
    timeHelper.sleep(5.0)


if __name__ == '__main__':
    main()
