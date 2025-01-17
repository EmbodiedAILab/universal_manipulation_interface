import sys
import os

ROOT_DIR = os.path.dirname(os.path.dirname(__file__))
sys.path.append(ROOT_DIR)
os.chdir(ROOT_DIR)

import glob
from umi.real_world.dh_modbus_gripper import dh_modbus_gripper
from time import sleep


def modbus_gripper():
    port = "/dev/ttyUSB0"  # please make sure you have the permission to access this port.
    baudrate = 115200
    initstate = 0
    g_state = 0
    force = 100
    speed = 100

    m_gripper = dh_modbus_gripper(port, baudrate)
    m_gripper.open()
    m_gripper.Initialization()
    print("Send grip init")

    while initstate != 1:
        initstate = m_gripper.GetInitState()
        sleep(0.2)

    m_gripper.SetTargetForce(force)
    m_gripper.SetTargetSpeed(speed)

    while True:
        g_state = 0
        m_gripper.SetTargetAbsPosition(0)
        while g_state == 0:
            g_state = m_gripper.GetGripState()
            print(f"init state: {m_gripper.GetInitState()}")
            print(f"gripper state: {m_gripper.GetGripState()}")
            print(f"target position: {m_gripper.GetCurrentAbsPosition()}")
            print(f"current position: {m_gripper.GetCurrentPosition()}")
            print(f"target force: {m_gripper.GetTargetForce()}")
            print(f"target speed: {m_gripper.GetTargetSpeed()}")
            sleep(0.2)

        g_state = 0
        m_gripper.SetTargetAbsPosition(0.08)
        while g_state == 0:
            g_state = m_gripper.GetGripState()
            print(f"init state: {m_gripper.GetInitState()}")
            print(f"gripper state: {m_gripper.GetGripState()}")
            print(f"target position: {m_gripper.GetCurrentAbsPosition()}")
            print(f"current position: {m_gripper.GetCurrentPosition()}")
            print(f"target force: {m_gripper.GetTargetForce()}")
            print(f"target speed: {m_gripper.GetTargetSpeed()}")
            sleep(0.2)
    m_gripper.close()


if __name__ == "__main__":
    modbus_gripper()