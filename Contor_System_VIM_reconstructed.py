import serial
from PyQt5 import QtWidgets, uic
from PyQt5.QtSerialPort import QSerialPort, QSerialPortInfo
from PyQt5.QtCore import QIODevice, QTimer
from QLed import QLed
import sys
import os
import numpy as np
import minimalmodbus
from pynput import keyboard
import time

app = QtWidgets.QApplication([])
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
ui = uic.loadUi(os.path.join(BASE_DIR, 'Interf.ui'))
timer = QTimer()

PortList = []
KeyFlag = False
ListenerFlag = False
Position = 0
TimerState = 0

# Status LEDs for each subsystem (Square shape, Green=OK, Red=Error)
LedStateManipulator = QLed()
LedStateLadle = QLed()
LedStateFrame = QLed()
LedStateBunker = QLed()
LedStateFlaps = QLed()
LedStateSeparator = QLed()
LedStateWheel = QLed()
LedStateMotorFrontL = QLed()
LedStateMotorFrontR = QLed()
LedStateMotorRearL = QLed()
LedStateMotorRearR = QLed()


# ---------------------------------------------------------------------------
# Connection / Disconnection
# ---------------------------------------------------------------------------

def ClickPortOpenButton():
    """Connect to all 11 Modbus devices or disconnect."""
    global ListenerFlag, Position

    if ui.PortOpenButton.text() == "СТАРТ":
        # ---------- Connect ----------
        # All devices share the same serial port / baud rate
        port = ui.PortNumberCombo.currentText()

        Dev1 = minimalmodbus.Instrument(port, 1)   # manipulator relay board
        Dev1.serial.baudrate = 38400
        Dev1.serial.timeout = 0.5

        Dev2 = minimalmodbus.Instrument(port, 2)   # ladle/bucket relay board
        Dev2.serial.baudrate = 38400
        Dev2.serial.timeout = 0.5

        Dev3 = minimalmodbus.Instrument(port, 3)   # frame relay board
        Dev3.serial.baudrate = 38400
        Dev3.serial.timeout = 0.5

        Dev4 = minimalmodbus.Instrument(port, 4)   # bunker relay board
        Dev4.serial.baudrate = 38400
        Dev4.serial.timeout = 0.5

        Dev5 = minimalmodbus.Instrument(port, 5)   # flaps relay board
        Dev5.serial.baudrate = 38400
        Dev5.serial.timeout = 0.5

        Dev6 = minimalmodbus.Instrument(port, 6)   # separator (BLDC)
        Dev6.serial.baudrate = 38400
        Dev6.serial.timeout = 0.5

        Dev7 = minimalmodbus.Instrument(port, 7)   # steering relay board
        Dev7.serial.baudrate = 38400
        Dev7.serial.timeout = 0.5

        Dev8 = minimalmodbus.Instrument(port, 8)   # FL motor BLDC
        Dev8.serial.baudrate = 38400
        Dev8.serial.timeout = 0.5

        Dev9 = minimalmodbus.Instrument(port, 9)   # FR motor BLDC
        Dev9.serial.baudrate = 38400
        Dev9.serial.timeout = 0.5

        Dev10 = minimalmodbus.Instrument(port, 10)  # RL motor BLDC
        Dev10.serial.baudrate = 38400
        Dev10.serial.timeout = 0.5

        Dev11 = minimalmodbus.Instrument(port, 11)  # RR motor BLDC
        Dev11.serial.baudrate = 38400
        Dev11.serial.timeout = 0.5

        time.sleep(0.1)

        # Initial UI button states
        ui.pushButtonDriveStop.setChecked(True)
        ui.pushButtonReverserUp.setChecked(True)
        ui.pushButtonReverserDown.setChecked(False)
        ui.pushButtonLadleUp.setChecked(False)
        ui.pushButtonFrameDown.setChecked(False)
        ui.pushButtonBunkerDown.setChecked(False)
        ui.pushButtonFlapsDown.setChecked(False)
        ui.pushButtonSeparatorStop.setChecked(True)
        ui.pushButtonSeparatorUp.setChecked(False)
        ui.pushButtonSeparatorUp.setEnabled(True)
        ui.pushButtonSeparatorDown.setChecked(False)
        ui.pushButtonSeparatorDown.setEnabled(True)

        # --- Probe Dev1 (manipulator) ---
        try:
            Dev1.read_register(0, 0, 3, False)   # FC03, reg 0
            LedStateManipulator.setOnColour(LedStateManipulator.Green)
            LedStateManipulator.setOffColour(LedStateManipulator.Grey)
            LedStateManipulator.setValue(True)
            ui.groupBoxControlManipulator.setEnabled(True)
        except minimalmodbus.ModbusException as err:
            LedStateManipulator.setOnColour(LedStateManipulator.Red)
            LedStateManipulator.setOffColour(LedStateManipulator.Grey)
            LedStateManipulator.setValue(True)
            print(err)

        # --- Probe Dev2 (ladle) ---
        try:
            Dev2.read_register(0, 0, 3, False)
            LedStateLadle.setOnColour(LedStateLadle.Green)
            LedStateLadle.setOffColour(LedStateLadle.Grey)
            LedStateLadle.setValue(True)
            ui.groupBoxControlLadle.setEnabled(True)
        except minimalmodbus.ModbusException as err:
            LedStateLadle.setOnColour(LedStateLadle.Red)
            LedStateLadle.setOffColour(LedStateLadle.Grey)
            LedStateLadle.setValue(True)
            print(err)

        # --- Probe Dev3 (frame) ---
        try:
            Dev3.read_register(0, 0, 3, False)
            LedStateFrame.setOnColour(LedStateFrame.Green)
            LedStateFrame.setOffColour(LedStateFrame.Grey)
            LedStateFrame.setValue(True)
            ui.groupBoxControlFrame.setEnabled(True)
        except minimalmodbus.ModbusException as err:
            LedStateFrame.setOnColour(LedStateFrame.Red)
            LedStateFrame.setOffColour(LedStateFrame.Grey)
            LedStateFrame.setValue(True)
            print(err)

        # --- Probe Dev4 (bunker) ---
        try:
            Dev4.read_register(0, 0, 3, False)
            LedStateBunker.setOnColour(LedStateBunker.Green)
            LedStateBunker.setOffColour(LedStateBunker.Grey)
            LedStateBunker.setValue(True)
            ui.groupBoxControlBunker.setEnabled(True)
        except minimalmodbus.ModbusException as err:
            LedStateBunker.setOnColour(LedStateBunker.Red)
            LedStateBunker.setOffColour(LedStateBunker.Grey)
            LedStateBunker.setValue(True)
            print(err)

        # --- Probe Dev5 (flaps) ---
        try:
            Dev5.read_register(0, 0, 3, False)
            LedStateFlaps.setOnColour(LedStateFlaps.Green)
            LedStateFlaps.setOffColour(LedStateFlaps.Grey)
            LedStateFlaps.setValue(True)
            ui.groupBoxControlFlaps.setEnabled(True)
        except minimalmodbus.ModbusException as err:
            LedStateFlaps.setOnColour(LedStateFlaps.Red)
            LedStateFlaps.setOffColour(LedStateFlaps.Grey)
            LedStateFlaps.setValue(True)
            print(err)

        # --- Init Dev6 (separator BLDC): enable coil 0 ON, direction coil 1 = 0, speed reg 0 = 0 ---
        try:
            Dev6.write_bit(0, 1, 5)        # FC05 coil 0 = 1 (enable)
            # NOTE: on disconnect the code does write_register(0,0,0,6,False),
            #       write_bit(1,0,5), write_bit(0,0,5)
            LedStateSeparator.setOnColour(LedStateSeparator.Green)
            LedStateSeparator.setOffColour(LedStateSeparator.Grey)
            LedStateSeparator.setValue(True)
            ui.groupBoxControlSeparator.setEnabled(True)
        except minimalmodbus.ModbusException as err:
            LedStateSeparator.setOnColour(LedStateSeparator.Red)
            LedStateSeparator.setOffColour(LedStateSeparator.Grey)
            LedStateSeparator.setValue(True)
            print(err)

        # --- Probe Dev7 (steering) ---
        try:
            Dev7.read_register(0, 0, 3, False)
            LedStateWheel.setOnColour(LedStateWheel.Green)
            LedStateWheel.setOffColour(LedStateWheel.Grey)
            LedStateWheel.setValue(True)
            ui.groupBoxControlSteering.setEnabled(True)
        except minimalmodbus.ModbusException as err:
            LedStateWheel.setOnColour(LedStateWheel.Red)
            LedStateWheel.setOffColour(LedStateWheel.Grey)
            LedStateWheel.setValue(True)
            print(err)

        # --- Init Dev8 (FL motor BLDC): enable coil 0 = 1 ---
        try:
            Dev8.write_bit(0, 1, 5)        # FC05 coil 0 = 1 (enable)
            LedStateMotorFrontL.setOnColour(LedStateMotorFrontL.Green)
            LedStateMotorFrontL.setOffColour(LedStateMotorFrontL.Grey)
            LedStateMotorFrontL.setValue(True)
            ui.groupBoxControlDrive.setEnabled(True)
            ui.checkBoxBlockMotorFrontL.setChecked(False)
            ui.checkBoxBlockMotorFrontL.setEnabled(True)
        except minimalmodbus.ModbusException as err:
            LedStateMotorFrontL.setOnColour(LedStateMotorFrontL.Red)
            LedStateMotorFrontL.setOffColour(LedStateMotorFrontL.Grey)
            LedStateMotorFrontL.setValue(True)
            ui.checkBoxBlockMotorFrontL.setChecked(True)
            ui.checkBoxBlockMotorFrontL.setEnabled(False)
            print(err)

        # --- Init Dev9 (FR motor BLDC): enable coil 0 = 1 ---
        try:
            Dev9.write_bit(0, 1, 5)
            LedStateMotorFrontR.setOnColour(LedStateMotorFrontR.Green)
            LedStateMotorFrontR.setOffColour(LedStateMotorFrontR.Grey)
            LedStateMotorFrontR.setValue(True)
            ui.groupBoxControlDrive.setEnabled(True)
            ui.checkBoxBlockMotorFrontR.setChecked(False)
            ui.checkBoxBlockMotorFrontR.setEnabled(True)
        except minimalmodbus.ModbusException as err:
            LedStateMotorFrontR.setOnColour(LedStateMotorFrontR.Red)
            LedStateMotorFrontR.setOffColour(LedStateMotorFrontR.Grey)
            LedStateMotorFrontR.setValue(True)
            ui.checkBoxBlockMotorFrontR.setChecked(True)
            ui.checkBoxBlockMotorFrontR.setEnabled(False)
            print(err)

        # --- Init Dev10 (RL motor BLDC): enable coil 0 = 1 ---
        try:
            Dev10.write_bit(0, 1, 5)
            LedStateMotorRearL.setOnColour(LedStateMotorRearL.Green)
            LedStateMotorRearL.setOffColour(LedStateMotorRearL.Grey)
            LedStateMotorRearL.setValue(True)
            ui.groupBoxControlDrive.setEnabled(True)
            ui.checkBoxBlockMotorRearL.setChecked(False)
            ui.checkBoxBlockMotorRearL.setEnabled(True)
        except minimalmodbus.ModbusException as err:
            LedStateMotorRearL.setOnColour(LedStateMotorRearL.Red)
            LedStateMotorRearL.setOffColour(LedStateMotorRearL.Grey)
            LedStateMotorRearL.setValue(True)
            ui.checkBoxBlockMotorRearL.setChecked(True)
            ui.checkBoxBlockMotorRearL.setEnabled(False)
            print(err)

        # --- Init Dev11 (RR motor BLDC): enable coil 0 = 1 ---
        try:
            Dev11.write_bit(0, 1, 5)
            LedStateMotorRearR.setOnColour(LedStateMotorRearR.Green)
            LedStateMotorRearR.setOffColour(LedStateMotorRearR.Grey)
            LedStateMotorRearR.setValue(True)
            ui.groupBoxControlDrive.setEnabled(True)
            ui.checkBoxBlockMotorRearR.setChecked(False)
            ui.checkBoxBlockMotorRearR.setEnabled(True)
        except minimalmodbus.ModbusException as err:
            LedStateMotorRearR.setOnColour(LedStateMotorRearR.Red)
            LedStateMotorRearR.setOffColour(LedStateMotorRearR.Grey)
            LedStateMotorRearR.setValue(True)
            ui.checkBoxBlockMotorRearR.setChecked(True)
            ui.checkBoxBlockMotorRearR.setEnabled(False)
            print(err)

        ListenerFlag = True
        timer.start(100)
        ui.PortOpenButton.setText("СТОП")

    elif ui.PortOpenButton.text() == "СТОП":
        # ---------- Disconnect – stop/zero every device that is enabled ----------

        if ui.groupBoxControlManipulator.isEnabled():
            try:
                Dev1.write_long(0, 0)          # zero both channels
            except minimalmodbus.ModbusException as err:
                print(err)

        if ui.groupBoxControlLadle.isEnabled():
            try:
                Dev2.write_long(0, 0)
            except minimalmodbus.ModbusException as err:
                print(err)

        if ui.groupBoxControlFrame.isEnabled():
            try:
                Dev3.write_long(0, 0)
            except minimalmodbus.ModbusException as err:
                print(err)

        if ui.groupBoxControlBunker.isEnabled():
            try:
                Dev4.write_long(0, 0)
            except minimalmodbus.ModbusException as err:
                print(err)

        if ui.groupBoxControlFlaps.isEnabled():
            try:
                Dev5.write_long(0, 0)
            except minimalmodbus.ModbusException as err:
                print(err)

        if ui.groupBoxControlSeparator.isEnabled():
            try:
                Dev6.write_register(0, 0, 0, 6, False)  # speed = 0, FC06
                Dev6.write_bit(1, 0, 5)                 # direction coil = 0
                Dev6.write_bit(0, 0, 5)                 # enable coil = 0 (disable)
            except minimalmodbus.ModbusException as err:
                print(err)

        if ui.groupBoxControlSteering.isEnabled():
            try:
                Dev7.write_long(0, 0)
            except minimalmodbus.ModbusException as err:
                print(err)

        if ui.checkBoxBlockMotorFrontL.isEnabled():
            try:
                Dev8.write_register(0, 0, 0, 6, False)  # speed = 0
                Dev8.write_bit(1, 0, 5)                 # direction = 0
                Dev8.write_bit(0, 0, 5)                 # enable = 0
            except minimalmodbus.ModbusException as err:
                print(err)

        if ui.checkBoxBlockMotorFrontR.isEnabled():
            try:
                Dev9.write_register(0, 0, 0, 6, False)
                Dev9.write_bit(1, 0, 5)
                Dev9.write_bit(0, 0, 5)
            except minimalmodbus.ModbusException as err:
                print(err)

        if ui.checkBoxBlockMotorRearL.isEnabled():
            try:
                Dev10.write_register(0, 0, 0, 6, False)
                Dev10.write_bit(1, 0, 5)
                Dev10.write_bit(0, 0, 5)
            except minimalmodbus.ModbusException as err:
                print(err)

        if ui.checkBoxBlockMotorRearR.isEnabled():
            try:
                Dev11.write_register(0, 0, 0, 6, False)
                Dev11.write_bit(1, 0, 5)
                Dev11.write_bit(0, 0, 5)
            except minimalmodbus.ModbusException as err:
                print(err)

        Dev1.serial.close()

        ListenerFlag = False
        Position = 0
        ui.progressBarLoad.setValue(Position)

        # Turn all status LEDs off
        LedStateManipulator.setValue(False)
        LedStateLadle.setValue(False)
        LedStateFrame.setValue(False)
        LedStateBunker.setValue(False)
        LedStateFlaps.setValue(False)
        LedStateSeparator.setValue(False)
        LedStateWheel.setValue(False)
        LedStateMotorFrontL.setValue(False)
        LedStateMotorFrontR.setValue(False)
        LedStateMotorRearL.setValue(False)
        LedStateMotorRearR.setValue(False)

        # Disable all control groups
        ui.groupBoxControlManipulator.setEnabled(False)
        ui.groupBoxControlLadle.setEnabled(False)
        ui.groupBoxControlFrame.setEnabled(False)
        ui.groupBoxControlBunker.setEnabled(False)
        ui.groupBoxControlFlaps.setEnabled(False)
        ui.groupBoxControlSeparator.setEnabled(False)
        ui.groupBoxControlSteering.setEnabled(False)
        ui.groupBoxControlDrive.setEnabled(False)

        ui.PortOpenButton.setText("СТАРТ")


# ---------------------------------------------------------------------------
# BLDC direction helpers  (called by reverser buttons)
# Direction arg: False = forward, True = reverse
# Writes to coil 1 (direction) via FC05
# ---------------------------------------------------------------------------

def DirectionControlDev8(Direction):
    """Set direction for Dev8 (FL motor). Direction: True=reverse, False=forward."""
    try:
        Dev8.write_bit(1, Direction, 5)    # FC05, coil 1
        LedStateMotorFrontL.setOnColour(LedStateMotorFrontL.Green)
        LedStateMotorFrontL.setOffColour(LedStateMotorFrontL.Grey)
        LedStateMotorFrontL.setValue(True)
    except minimalmodbus.ModbusException as err:
        LedStateMotorFrontL.setOnColour(LedStateMotorFrontL.Red)
        LedStateMotorFrontL.setOffColour(LedStateMotorFrontL.Grey)
        LedStateMotorFrontL.setValue(True)
        print(err)


def DirectionControlDev9(Direction):
    """Set direction for Dev9 (FR motor). Direction: True=reverse, False=forward."""
    try:
        Dev9.write_bit(1, Direction, 5)
        LedStateMotorFrontR.setOnColour(LedStateMotorFrontR.Green)
        LedStateMotorFrontR.setOffColour(LedStateMotorFrontR.Grey)
        LedStateMotorFrontR.setValue(True)
    except minimalmodbus.ModbusException as err:
        LedStateMotorFrontR.setOnColour(LedStateMotorFrontR.Red)
        LedStateMotorFrontR.setOffColour(LedStateMotorFrontR.Grey)
        LedStateMotorFrontR.setValue(True)
        print(err)


def DirectionControlDev10(Direction):
    """Set direction for Dev10 (RL motor). Direction: True=reverse, False=forward."""
    try:
        Dev10.write_bit(1, Direction, 5)
        LedStateMotorRearL.setOnColour(LedStateMotorRearL.Green)
        LedStateMotorRearL.setOffColour(LedStateMotorRearL.Grey)
        LedStateMotorRearL.setValue(True)
    except minimalmodbus.ModbusException as err:
        LedStateMotorRearL.setOnColour(LedStateMotorRearL.Red)
        LedStateMotorRearL.setOffColour(LedStateMotorRearL.Grey)
        LedStateMotorRearL.setValue(True)
        print(err)


def DirectionControlDev11(Direction):
    """Set direction for Dev11 (RR motor). Direction: True=reverse, False=forward."""
    try:
        Dev11.write_bit(1, Direction, 5)
        LedStateMotorRearR.setOnColour(LedStateMotorRearR.Green)
        LedStateMotorRearR.setOffColour(LedStateMotorRearR.Grey)
        LedStateMotorRearR.setValue(True)
    except minimalmodbus.ModbusException as err:
        LedStateMotorRearR.setOnColour(LedStateMotorRearR.Red)
        LedStateMotorRearR.setOffColour(LedStateMotorRearR.Grey)
        LedStateMotorRearR.setValue(True)
        print(err)


# ---------------------------------------------------------------------------
# BLDC speed helpers  (called by TimeoutTimer and keyboard shortcuts)
# Writes to register 0 (speed 0-255) via FC06
# ---------------------------------------------------------------------------

def SpeedControlDev8(Speed):
    """Set speed for Dev8 (FL motor). Speed: 0-255."""
    try:
        Dev8.write_register(0, Speed, 0, 6, False)   # FC06, reg 0, decimals 0
        LedStateMotorFrontL.setOnColour(LedStateMotorFrontL.Green)
        LedStateMotorFrontL.setOffColour(LedStateMotorFrontL.Grey)
        LedStateMotorFrontL.setValue(True)
    except minimalmodbus.ModbusException as err:
        LedStateMotorFrontL.setOnColour(LedStateMotorFrontL.Red)
        LedStateMotorFrontL.setOffColour(LedStateMotorFrontL.Grey)
        LedStateMotorFrontL.setValue(True)
        print(err)


def SpeedControlDev9(Speed):
    """Set speed for Dev9 (FR motor). Speed: 0-255."""
    try:
        Dev9.write_register(0, Speed, 0, 6, False)
        LedStateMotorFrontR.setOnColour(LedStateMotorFrontR.Green)
        LedStateMotorFrontR.setOffColour(LedStateMotorFrontR.Grey)
        LedStateMotorFrontR.setValue(True)
    except minimalmodbus.ModbusException as err:
        LedStateMotorFrontR.setOnColour(LedStateMotorFrontR.Red)
        LedStateMotorFrontR.setOffColour(LedStateMotorFrontR.Grey)
        LedStateMotorFrontR.setValue(True)
        print(err)


def SpeedControlDev10(Speed):
    """Set speed for Dev10 (RL motor). Speed: 0-255."""
    try:
        Dev10.write_register(0, Speed, 0, 6, False)
        LedStateMotorRearL.setOnColour(LedStateMotorRearL.Green)
        LedStateMotorRearL.setOffColour(LedStateMotorRearL.Grey)
        LedStateMotorRearL.setValue(True)
    except minimalmodbus.ModbusException as err:
        LedStateMotorRearL.setOnColour(LedStateMotorRearL.Red)
        LedStateMotorRearL.setOffColour(LedStateMotorRearL.Grey)
        LedStateMotorRearL.setValue(True)
        print(err)


def SpeedControlDev11(Speed):
    """Set speed for Dev11 (RR motor). Speed: 0-255."""
    try:
        Dev11.write_register(0, Speed, 0, 6, False)
        LedStateMotorRearR.setOnColour(LedStateMotorRearR.Green)
        LedStateMotorRearR.setOffColour(LedStateMotorRearR.Grey)
        LedStateMotorRearR.setValue(True)
    except minimalmodbus.ModbusException as err:
        LedStateMotorRearR.setOnColour(LedStateMotorRearR.Red)
        LedStateMotorRearR.setOffColour(LedStateMotorRearR.Grey)
        LedStateMotorRearR.setValue(True)
        print(err)


# ---------------------------------------------------------------------------
# Manipulator (Dev1, address 1) – relay board
# Register 0 = channel 0 (UP/DOWN arm axis), register 1 = channel 1 (LEFT/RIGHT axis)
# Value 1 = forward, value 2 = reverse, value 0 = stop
# FC06  (write_register decimals=0, fc=6, signed=False)
# The relay board accepts a 32-bit "long" write (write_long) to zero both
# channels at once during disconnect.
# ---------------------------------------------------------------------------

def pressedpushButtonManipulatorUp():
    ui.pushButtonManipulatorUp.setChecked(True)
    try:
        Dev1.write_register(0, 1, 0, 6, False)   # channel 0 = forward (up)
        LedStateManipulator.setOnColour(LedStateManipulator.Green)
        LedStateManipulator.setOffColour(LedStateManipulator.Grey)
        LedStateManipulator.setValue(True)
    except minimalmodbus.ModbusException as err:
        LedStateManipulator.setOnColour(LedStateManipulator.Red)
        LedStateManipulator.setOffColour(LedStateManipulator.Grey)
        LedStateManipulator.setValue(True)
        print(err)


def releasedpushButtonManipulatorUp():
    ui.pushButtonManipulatorUp.setChecked(False)
    try:
        Dev1.write_register(0, 0, 0, 6, False)   # channel 0 = stop
        LedStateManipulator.setOnColour(LedStateManipulator.Green)
        LedStateManipulator.setOffColour(LedStateManipulator.Grey)
        LedStateManipulator.setValue(True)
    except minimalmodbus.ModbusException as err:
        LedStateManipulator.setOnColour(LedStateManipulator.Red)
        LedStateManipulator.setOffColour(LedStateManipulator.Grey)
        LedStateManipulator.setValue(True)
        print(err)


def pressedpushButtonManipulatorDown():
    ui.pushButtonManipulatorDown.setChecked(True)
    try:
        Dev1.write_register(0, 2, 0, 6, False)   # channel 0 = reverse (down)
        LedStateManipulator.setOnColour(LedStateManipulator.Green)
        LedStateManipulator.setOffColour(LedStateManipulator.Grey)
        LedStateManipulator.setValue(True)
    except minimalmodbus.ModbusException as err:
        LedStateManipulator.setOnColour(LedStateManipulator.Red)
        LedStateManipulator.setOffColour(LedStateManipulator.Grey)
        LedStateManipulator.setValue(True)
        print(err)


def releasedpushButtonManipulatorDown():
    ui.pushButtonManipulatorDown.setChecked(False)
    try:
        Dev1.write_register(0, 0, 0, 6, False)   # channel 0 = stop
        LedStateManipulator.setOnColour(LedStateManipulator.Green)
        LedStateManipulator.setOffColour(LedStateManipulator.Grey)
        LedStateManipulator.setValue(True)
    except minimalmodbus.ModbusException as err:
        LedStateManipulator.setOnColour(LedStateManipulator.Red)
        LedStateManipulator.setOffColour(LedStateManipulator.Grey)
        LedStateManipulator.setValue(True)
        print(err)


def pressedpushButtonManipulatorLeft():
    ui.pushButtonManipulatorLeft.setChecked(True)
    try:
        Dev1.write_register(1, 1, 0, 6, False)   # channel 1 = forward (left)
        LedStateManipulator.setOnColour(LedStateManipulator.Green)
        LedStateManipulator.setOffColour(LedStateManipulator.Grey)
        LedStateManipulator.setValue(True)
    except minimalmodbus.ModbusException as err:
        LedStateManipulator.setOnColour(LedStateManipulator.Red)
        LedStateManipulator.setOffColour(LedStateManipulator.Grey)
        LedStateManipulator.setValue(True)
        print(err)


def releasedpushButtonManipulatorLeft():
    ui.pushButtonManipulatorLeft.setChecked(False)
    try:
        Dev1.write_register(1, 0, 0, 6, False)   # channel 1 = stop
        LedStateManipulator.setOnColour(LedStateManipulator.Green)
        LedStateManipulator.setOffColour(LedStateManipulator.Grey)
        LedStateManipulator.setValue(True)
    except minimalmodbus.ModbusException as err:
        LedStateManipulator.setOnColour(LedStateManipulator.Red)
        LedStateManipulator.setOffColour(LedStateManipulator.Grey)
        LedStateManipulator.setValue(True)
        print(err)


def pressedpushButtonManipulatorRight():
    ui.pushButtonManipulatorRight.setChecked(True)
    try:
        Dev1.write_register(1, 2, 0, 6, False)   # channel 1 = reverse (right)
        LedStateManipulator.setOnColour(LedStateManipulator.Green)
        LedStateManipulator.setOffColour(LedStateManipulator.Grey)
        LedStateManipulator.setValue(True)
    except minimalmodbus.ModbusException as err:
        LedStateManipulator.setOnColour(LedStateManipulator.Red)
        LedStateManipulator.setOffColour(LedStateManipulator.Grey)
        LedStateManipulator.setValue(True)
        print(err)


def releasedpushButtonManipulatorRight():
    ui.pushButtonManipulatorRight.setChecked(False)
    try:
        Dev1.write_register(1, 0, 0, 6, False)   # channel 1 = stop
        LedStateManipulator.setOnColour(LedStateManipulator.Green)
        LedStateManipulator.setOffColour(LedStateManipulator.Grey)
        LedStateManipulator.setValue(True)
    except minimalmodbus.ModbusException as err:
        LedStateManipulator.setOnColour(LedStateManipulator.Red)
        LedStateManipulator.setOffColour(LedStateManipulator.Grey)
        LedStateManipulator.setValue(True)
        print(err)


# ---------------------------------------------------------------------------
# Ladle/bucket (Dev2, address 2) – relay board
# Uses write_long(0, value) – a 32-bit value that encodes both channels.
# 65537  = 0x00010001 => channel0=1 (forward/up),   channel1=1 (forward)
# 131074 = 0x00020002 => channel0=2 (reverse/down), channel1=2 (reverse)
# 0      = 0x00000000 => both channels = stop
# (The relay board appears to interpret a 32-bit long across reg 0 and reg 1)
# ---------------------------------------------------------------------------

def clickedpushButtonLadleUp():
    ui.pushButtonLadleUp.setChecked(True)
    try:
        Dev2.write_long(0, 65537)    # both channels = 1 (up/forward)
        LedStateLadle.setOnColour(LedStateLadle.Green)
        LedStateLadle.setOffColour(LedStateLadle.Grey)
        LedStateLadle.setValue(True)
    except minimalmodbus.ModbusException as err:
        LedStateLadle.setOnColour(LedStateLadle.Red)
        LedStateLadle.setOffColour(LedStateLadle.Grey)
        LedStateLadle.setValue(True)
        print(err)


def pressedpushButtonLadleDown():
    ui.pushButtonLadleUp.setChecked(False)
    ui.pushButtonLadleDown.setChecked(True)
    try:
        Dev2.write_long(0, 131074)   # both channels = 2 (down/reverse)
        LedStateLadle.setOnColour(LedStateLadle.Green)
        LedStateLadle.setOffColour(LedStateLadle.Grey)
        LedStateLadle.setValue(True)
    except minimalmodbus.ModbusException as err:
        LedStateLadle.setOnColour(LedStateLadle.Red)
        LedStateLadle.setOffColour(LedStateLadle.Grey)
        LedStateLadle.setValue(True)
        print(err)


def releasedpushButtonLadleDown():
    ui.pushButtonLadleUp.setChecked(False)
    ui.pushButtonLadleDown.setChecked(False)
    try:
        Dev2.write_long(0, 0)        # both channels = 0 (stop)
        LedStateLadle.setOnColour(LedStateLadle.Green)
        LedStateLadle.setOffColour(LedStateLadle.Grey)
        LedStateLadle.setValue(True)
    except minimalmodbus.ModbusException as err:
        LedStateLadle.setOnColour(LedStateLadle.Red)
        LedStateLadle.setOffColour(LedStateLadle.Grey)
        LedStateLadle.setValue(True)
        print(err)


# ---------------------------------------------------------------------------
# Frame (Dev3, address 3) – relay board  (same write_long encoding as Dev2)
# ---------------------------------------------------------------------------

def clickedpushButtonFrameDown():
    ui.pushButtonFrameDown.setChecked(True)
    try:
        Dev3.write_long(0, 131074)   # both channels = 2 (down)
        LedStateFrame.setOnColour(LedStateFrame.Green)
        LedStateFrame.setOffColour(LedStateFrame.Grey)
        LedStateFrame.setValue(True)
    except minimalmodbus.ModbusException as err:
        LedStateFrame.setOnColour(LedStateFrame.Red)
        LedStateFrame.setOffColour(LedStateFrame.Grey)
        LedStateFrame.setValue(True)
        print(err)


def pressedpushButtonFrameUp():
    ui.pushButtonFrameDown.setChecked(False)
    ui.pushButtonFrameUp.setChecked(True)
    try:
        Dev3.write_long(0, 65537)    # both channels = 1 (up)
        LedStateFrame.setOnColour(LedStateFrame.Green)
        LedStateFrame.setOffColour(LedStateFrame.Grey)
        LedStateFrame.setValue(True)
    except minimalmodbus.ModbusException as err:
        LedStateFrame.setOnColour(LedStateFrame.Red)
        LedStateFrame.setOffColour(LedStateFrame.Grey)
        LedStateFrame.setValue(True)
        print(err)


def releasedpushButtonFrameUp():
    ui.pushButtonFrameDown.setChecked(False)
    ui.pushButtonFrameUp.setChecked(False)
    try:
        Dev3.write_long(0, 0)        # stop
        LedStateFrame.setOnColour(LedStateFrame.Green)
        LedStateFrame.setOffColour(LedStateFrame.Grey)
        LedStateFrame.setValue(True)
    except minimalmodbus.ModbusException as err:
        LedStateFrame.setOnColour(LedStateFrame.Red)
        LedStateFrame.setOffColour(LedStateFrame.Grey)
        LedStateFrame.setValue(True)
        print(err)


# ---------------------------------------------------------------------------
# Bunker (Dev4, address 4) – relay board
# ---------------------------------------------------------------------------

def clickedpushButtonBunkerDown():
    ui.pushButtonBunkerDown.setChecked(True)
    try:
        Dev4.write_long(0, 131074)
        LedStateBunker.setOnColour(LedStateBunker.Green)
        LedStateBunker.setOffColour(LedStateBunker.Grey)
        LedStateBunker.setValue(True)
    except minimalmodbus.ModbusException as err:
        LedStateBunker.setOnColour(LedStateBunker.Red)
        LedStateBunker.setOffColour(LedStateBunker.Grey)
        LedStateBunker.setValue(True)
        print(err)


def pressedpushButtonBunkerUp():
    ui.pushButtonBunkerDown.setChecked(False)
    ui.pushButtonBunkerUp.setChecked(True)
    try:
        Dev4.write_long(0, 65537)
        LedStateBunker.setOnColour(LedStateBunker.Green)
        LedStateBunker.setOffColour(LedStateBunker.Grey)
        LedStateBunker.setValue(True)
    except minimalmodbus.ModbusException as err:
        LedStateBunker.setOnColour(LedStateBunker.Red)
        LedStateBunker.setOffColour(LedStateBunker.Grey)
        LedStateBunker.setValue(True)
        print(err)


def releasedpushButtonBunkerUp():
    ui.pushButtonBunkerDown.setChecked(False)
    ui.pushButtonBunkerUp.setChecked(False)
    try:
        Dev4.write_long(0, 0)
        LedStateBunker.setOnColour(LedStateBunker.Green)
        LedStateBunker.setOffColour(LedStateBunker.Grey)
        LedStateBunker.setValue(True)
    except minimalmodbus.ModbusException as err:
        LedStateBunker.setOnColour(LedStateBunker.Red)
        LedStateBunker.setOffColour(LedStateBunker.Grey)
        LedStateBunker.setValue(True)
        print(err)


# ---------------------------------------------------------------------------
# Flaps (Dev5, address 5) – relay board
# ---------------------------------------------------------------------------

def clickedpushButtonFlapsDown():
    ui.pushButtonFlapsDown.setChecked(True)
    try:
        Dev5.write_long(0, 131074)
        LedStateFlaps.setOnColour(LedStateFlaps.Green)
        LedStateFlaps.setOffColour(LedStateFlaps.Grey)
        LedStateFlaps.setValue(True)
    except minimalmodbus.ModbusException as err:
        LedStateFlaps.setOnColour(LedStateFlaps.Red)
        LedStateFlaps.setOffColour(LedStateFlaps.Grey)
        LedStateFlaps.setValue(True)
        print(err)


def pressedpushButtonFlapsUp():
    ui.pushButtonFlapsDown.setChecked(False)
    ui.pushButtonFlapsUp.setChecked(True)
    try:
        Dev5.write_long(0, 65537)
        LedStateFlaps.setOnColour(LedStateFlaps.Green)
        LedStateFlaps.setOffColour(LedStateFlaps.Grey)
        LedStateFlaps.setValue(True)
    except minimalmodbus.ModbusException as err:
        LedStateFlaps.setOnColour(LedStateFlaps.Red)
        LedStateFlaps.setOffColour(LedStateFlaps.Grey)
        LedStateFlaps.setValue(True)
        print(err)


def releasedpushButtonFlapsUp():
    ui.pushButtonFlapsDown.setChecked(False)
    ui.pushButtonFlapsUp.setChecked(False)
    try:
        Dev5.write_long(0, 0)
        LedStateFlaps.setOnColour(LedStateFlaps.Green)
        LedStateFlaps.setOffColour(LedStateFlaps.Grey)
        LedStateFlaps.setValue(True)
    except minimalmodbus.ModbusException as err:
        LedStateFlaps.setOnColour(LedStateFlaps.Red)
        LedStateFlaps.setOffColour(LedStateFlaps.Grey)
        LedStateFlaps.setValue(True)
        print(err)


# ---------------------------------------------------------------------------
# Separator (Dev6, address 6) – BLDC board
# Coil 1 (FC05) = direction: 0=forward(up), 1=reverse(down)
# Register 0 (FC06) = speed 0-255;  speed 255 = full
# ---------------------------------------------------------------------------

def clickedpushButtonSeparatorUp():
    ui.pushButtonSeparatorUp.setChecked(True)
    ui.pushButtonSeparatorDown.setChecked(False)
    ui.pushButtonSeparatorStop.setChecked(False)
    ui.pushButtonSeparatorDown.setEnabled(False)   # lock out down while moving up
    try:
        Dev6.write_bit(1, 0, 5)                    # direction = forward (up)
        Dev6.write_register(0, 255, 0, 6, False)   # speed = 255 (full)
        LedStateSeparator.setOnColour(LedStateSeparator.Green)
        LedStateSeparator.setOffColour(LedStateSeparator.Grey)
        LedStateSeparator.setValue(True)
    except minimalmodbus.ModbusException as err:
        LedStateSeparator.setOnColour(LedStateSeparator.Red)
        LedStateSeparator.setOffColour(LedStateSeparator.Grey)
        LedStateSeparator.setValue(True)
        print(err)


def clickedpushButtonSeparatorDown():
    ui.pushButtonSeparatorUp.setChecked(False)
    ui.pushButtonSeparatorDown.setChecked(True)
    ui.pushButtonSeparatorStop.setChecked(False)
    ui.pushButtonSeparatorUp.setEnabled(False)     # lock out up while moving down
    try:
        Dev6.write_bit(1, 1, 5)                    # direction = reverse (down)
        Dev6.write_register(0, 255, 0, 6, False)   # speed = 255 (full)
        LedStateSeparator.setOnColour(LedStateSeparator.Green)
        LedStateSeparator.setOffColour(LedStateSeparator.Grey)
        LedStateSeparator.setValue(True)
    except minimalmodbus.ModbusException as err:
        LedStateSeparator.setOnColour(LedStateSeparator.Red)
        LedStateSeparator.setOffColour(LedStateSeparator.Grey)
        LedStateSeparator.setValue(True)
        print(err)


def clickedpushButtonSeparatorStop():
    ui.pushButtonSeparatorUp.setChecked(False)
    ui.pushButtonSeparatorDown.setChecked(False)
    ui.pushButtonSeparatorStop.setChecked(True)
    ui.pushButtonSeparatorDown.setEnabled(True)
    ui.pushButtonSeparatorUp.setEnabled(True)
    try:
        Dev6.write_register(0, 0, 0, 6, False)     # speed = 0 (stop)
        LedStateSeparator.setOnColour(LedStateSeparator.Green)
        LedStateSeparator.setOffColour(LedStateSeparator.Grey)
        LedStateSeparator.setValue(True)
    except minimalmodbus.ModbusException as err:
        LedStateSeparator.setOnColour(LedStateSeparator.Red)
        LedStateSeparator.setOffColour(LedStateSeparator.Grey)
        LedStateSeparator.setValue(True)
        print(err)


# ---------------------------------------------------------------------------
# Steering (Dev7, address 7) – relay board
# Register 0 channel 0: 1=left, 2=right, 0=stop  (FC06)
# ---------------------------------------------------------------------------

def pressedpushButtonSteeringLeft():
    ui.pushButtonSteeringLeft.setChecked(True)
    try:
        Dev7.write_register(0, 1, 0, 6, False)    # channel 0 = forward (left)
        LedStateWheel.setOnColour(LedStateWheel.Green)
        LedStateWheel.setOffColour(LedStateWheel.Grey)
        LedStateWheel.setValue(True)
    except minimalmodbus.ModbusException as err:
        LedStateWheel.setOnColour(LedStateWheel.Red)
        LedStateWheel.setOffColour(LedStateWheel.Grey)
        LedStateWheel.setValue(True)
        print(err)


def releasedpushButtonSteeringLeft():
    ui.pushButtonSteeringLeft.setChecked(False)
    try:
        Dev7.write_register(0, 0, 0, 6, False)    # stop
        LedStateWheel.setOnColour(LedStateWheel.Green)
        LedStateWheel.setOffColour(LedStateWheel.Grey)
        LedStateWheel.setValue(True)
    except minimalmodbus.ModbusException as err:
        LedStateWheel.setOnColour(LedStateWheel.Red)
        LedStateWheel.setOffColour(LedStateWheel.Grey)
        LedStateWheel.setValue(True)
        print(err)


def pressedpushButtonSteeringRight():
    ui.pushButtonSteeringRight.setChecked(True)
    try:
        Dev7.write_register(0, 2, 0, 6, False)    # channel 0 = reverse (right)
        LedStateWheel.setOnColour(LedStateWheel.Green)
        LedStateWheel.setOffColour(LedStateWheel.Grey)
        LedStateWheel.setValue(True)
    except minimalmodbus.ModbusException as err:
        LedStateWheel.setOnColour(LedStateWheel.Red)
        LedStateWheel.setOffColour(LedStateWheel.Grey)
        LedStateWheel.setValue(True)
        print(err)


def releasedpushButtonSteeringRight():
    ui.pushButtonSteeringRight.setChecked(False)
    try:
        Dev7.write_register(0, 0, 0, 6, False)    # stop
        LedStateWheel.setOnColour(LedStateWheel.Green)
        LedStateWheel.setOffColour(LedStateWheel.Grey)
        LedStateWheel.setValue(True)
    except minimalmodbus.ModbusException as err:
        LedStateWheel.setOnColour(LedStateWheel.Red)
        LedStateWheel.setOffColour(LedStateWheel.Grey)
        LedStateWheel.setValue(True)
        print(err)


# ---------------------------------------------------------------------------
# Reverser buttons – set direction for all unlocked wheel motors
# ReverserUp  = forward  (Direction=False)
# ReverserDown = reverse  (Direction=True)
# ---------------------------------------------------------------------------

def clickedpushButtonReverserUp():
    """Set forward direction for all enabled drive motors."""
    ui.pushButtonReverserUp.setChecked(True)
    ui.pushButtonReverserDown.setChecked(False)
    if ui.checkBoxBlockMotorFrontL.isChecked() == False:
        DirectionControlDev8(False)
    if ui.checkBoxBlockMotorFrontR.isChecked() == False:
        DirectionControlDev9(False)
    if ui.checkBoxBlockMotorRearL.isChecked() == False:
        DirectionControlDev10(False)
    if ui.checkBoxBlockMotorRearR.isChecked() == False:
        DirectionControlDev11(False)


def clickedpushButtonReverserDown():
    """Set reverse direction for all enabled drive motors."""
    ui.pushButtonReverserDown.setChecked(True)
    ui.pushButtonReverserUp.setChecked(False)
    if ui.checkBoxBlockMotorFrontL.isChecked() == False:
        DirectionControlDev8(True)
    if ui.checkBoxBlockMotorFrontR.isChecked() == False:
        DirectionControlDev9(True)
    if ui.checkBoxBlockMotorRearL.isChecked() == False:
        DirectionControlDev10(True)
    if ui.checkBoxBlockMotorRearR.isChecked() == False:
        DirectionControlDev11(True)


# ---------------------------------------------------------------------------
# Position / speed ramp control buttons
# TimerState values:
#   0 = idle  (no ramp)
#   1 = ramp up   (Position increases by 5 per tick toward 255)
#   2 = ramp down (Position decreases by 5 per tick toward 0)
#   3 = stop  (Position = 0 immediately)
# ---------------------------------------------------------------------------

def pressedpushButtonPositionSet():
    """Start speed ramp-UP. Disables reverser controls while ramping."""
    ui.pushButtonPositionSet.setChecked(True)
    ui.pushButtonDriveStop.setChecked(False)
    ui.groupBoxControlReverser.setEnabled(False)
    TimerState = 1


def releasedpushButtonPositionSet():
    ui.pushButtonPositionSet.setChecked(False)
    TimerState = 0


def pressedpushButtonPositionReset():
    """Start speed ramp-DOWN. Disables reverser controls while ramping."""
    ui.pushButtonPositionReset.setChecked(True)
    ui.pushButtonDriveStop.setChecked(False)
    ui.groupBoxControlReverser.setEnabled(False)
    TimerState = 2


def releasedpushButtonPositionReset():
    ui.pushButtonPositionReset.setChecked(False)
    TimerState = 0


def pressedpushButtonDriveStop():
    """Immediately stop all motors (TimerState=3). Re-enables reverser."""
    ui.pushButtonDriveStop.setChecked(True)
    ui.groupBoxControlReverser.setEnabled(True)
    TimerState = 3


def releasedpushButtonDriveStop():
    ui.pushButtonDriveStop.setChecked(True)   # keep checked on release
    TimerState = 0


# ---------------------------------------------------------------------------
# Keyboard handler – pynput Listener
# Key map (on_press → action, on_release → stop/reset):
#   t  → ManipulatorUp (hold)
#   g  → ManipulatorDown (hold)
#   f  → ManipulatorLeft (hold)
#   h  → ManipulatorRight (hold)
#   r  → LadleUp (click)
#   y  → LadleDown (hold)
#   u  → FrameUp (hold)
#   j  → FrameDown (click)
#   o  → BunkerUp (hold)
#   i  → BunkerDown (click)
#   l  → FlapsUp (hold)
#   k  → FlapsDown (click)
#   b  → SeparatorUp (toggle – only if SeparatorDown is not active)
#   m  → SeparatorDown (toggle – only if SeparatorUp is not active)
#   n  → SeparatorStop (click)
#   a  → SteeringLeft (hold)
#   d  → SteeringRight (hold)
#   w  → PositionSet (ramp up, hold)
#   s  → PositionReset (ramp down, hold)
#   x  → DriveStop (hold)
#   q  → ReverserUp (if reverser enabled)
#   e  → ReverserDown (if reverser enabled)
# KeyFlag prevents key-repeat from firing the action multiple times per press.
# ---------------------------------------------------------------------------

def on_press(key):
    global KeyFlag
    try:
        if ListenerFlag == True and KeyFlag == False:
            if key.char == 't':
                pressedpushButtonManipulatorUp()
                KeyFlag = True
            elif key.char == 'g':
                pressedpushButtonManipulatorDown()
                KeyFlag = True
            elif key.char == 'f':
                pressedpushButtonManipulatorLeft()
                KeyFlag = True
            elif key.char == 'h':
                pressedpushButtonManipulatorRight()
                KeyFlag = True
            elif key.char == 'r':
                clickedpushButtonLadleUp()
                KeyFlag = True
            elif key.char == 'y':
                pressedpushButtonLadleDown()
                KeyFlag = True
            elif key.char == 'u':
                pressedpushButtonFrameUp()
                KeyFlag = True
            elif key.char == 'j':
                clickedpushButtonFrameDown()
                KeyFlag = True
            elif key.char == 'o':
                pressedpushButtonBunkerUp()
                KeyFlag = True
            elif key.char == 'i':
                clickedpushButtonBunkerDown()
                KeyFlag = True
            elif key.char == 'l':
                pressedpushButtonFlapsUp()
                KeyFlag = True
            elif key.char == 'k':
                clickedpushButtonFlapsDown()
                KeyFlag = True
            elif key.char == 'b':
                # Only trigger SeparatorUp if SeparatorDown is NOT currently active
                if ui.pushButtonSeparatorDown.isChecked() != True:
                    clickedpushButtonSeparatorUp()
                    KeyFlag = True
            elif key.char == 'm':
                # Only trigger SeparatorDown if SeparatorUp is NOT currently active
                if ui.pushButtonSeparatorUp.isChecked() != True:
                    clickedpushButtonSeparatorDown()
                    KeyFlag = True
            elif key.char == 'n':
                clickedpushButtonSeparatorStop()
                KeyFlag = True
            elif key.char == 'a':
                pressedpushButtonSteeringLeft()
                KeyFlag = True
            elif key.char == 'd':
                pressedpushButtonSteeringRight()
                KeyFlag = True
            elif key.char == 'w':
                pressedpushButtonPositionSet()
                KeyFlag = True
            elif key.char == 's':
                pressedpushButtonPositionReset()
                KeyFlag = True
            elif key.char == 'x':
                pressedpushButtonDriveStop()
                KeyFlag = True
            elif key.char == 'q':
                if ui.groupBoxControlReverser.isEnabled():
                    clickedpushButtonReverserUp()
                KeyFlag = True
            elif key.char == 'e':
                if ui.groupBoxControlReverser.isEnabled():
                    clickedpushButtonReverserDown()
                KeyFlag = True
    except AttributeError as err:
        print(err)


def on_release(key):
    global KeyFlag
    try:
        if ListenerFlag == True:
            if key.char == 't':
                releasedpushButtonManipulatorUp()
                KeyFlag = False
            elif key.char == 'g':
                releasedpushButtonManipulatorDown()
                KeyFlag = False
            elif key.char == 'f':
                releasedpushButtonManipulatorLeft()
                KeyFlag = False
            elif key.char == 'h':
                releasedpushButtonManipulatorRight()
                KeyFlag = False
            elif key.char == 'r':
                KeyFlag = False          # LadleUp is click-only, no release action
            elif key.char == 'y':
                releasedpushButtonLadleDown()
                KeyFlag = False
            elif key.char == 'u':
                releasedpushButtonFrameUp()
                KeyFlag = False
            elif key.char == 'j':
                KeyFlag = False          # FrameDown is click-only
            elif key.char == 'o':
                releasedpushButtonBunkerUp()
                KeyFlag = False
            elif key.char == 'i':
                KeyFlag = False          # BunkerDown is click-only
            elif key.char == 'l':
                releasedpushButtonFlapsUp()
                KeyFlag = False
            elif key.char == 'k':
                KeyFlag = False          # FlapsDown is click-only
            elif key.char == 'b':
                KeyFlag = False
            elif key.char == 'm':
                KeyFlag = False
            elif key.char == 'n':
                KeyFlag = False
            elif key.char == 'a':
                releasedpushButtonSteeringLeft()
                KeyFlag = False
            elif key.char == 'd':
                releasedpushButtonSteeringRight()
                KeyFlag = False
            elif key.char == 'w':
                releasedpushButtonPositionSet()
                KeyFlag = False
            elif key.char == 's':
                releasedpushButtonPositionReset()
                KeyFlag = False
            elif key.char == 'x':
                releasedpushButtonDriveStop()
                KeyFlag = False
            elif key.char == 'q':
                KeyFlag = False
            elif key.char == 'e':
                KeyFlag = False
    except AttributeError as err:
        print(err)


# ---------------------------------------------------------------------------
# Timer callback – drives the speed ramp
# Called every 100 ms by QTimer.
# TimerState 1: ramp Position up from 0→255 in steps of 5, send to all active motors
# TimerState 2: ramp Position down from current→0 in steps of 5
# TimerState 3: set Position=0, immediately send stop to all active motors
# ---------------------------------------------------------------------------

def TimeoutTimer():
    global Position, TimerState

    if TimerState == 0:
        return

    elif TimerState == 1:
        # Ramp UP
        if Position < 255:
            Position += 5
            ui.progressBarLoad.setValue(Position)
            if ui.checkBoxBlockMotorFrontL.isChecked() == False:
                SpeedControlDev8(Position)
            if ui.checkBoxBlockMotorFrontR.isChecked() == False:
                SpeedControlDev9(Position)
            if ui.checkBoxBlockMotorRearL.isChecked() == False:
                SpeedControlDev10(Position)
            if ui.checkBoxBlockMotorRearR.isChecked() == False:
                SpeedControlDev11(Position)
            print(Position)

    elif TimerState == 2:
        # Ramp DOWN
        if Position > 0:
            Position -= 5
            ui.progressBarLoad.setValue(Position)
            if ui.checkBoxBlockMotorFrontL.isChecked() == False:
                SpeedControlDev8(Position)
            if ui.checkBoxBlockMotorFrontR.isChecked() == False:
                SpeedControlDev9(Position)
            if ui.checkBoxBlockMotorRearL.isChecked() == False:
                SpeedControlDev10(Position)
            if ui.checkBoxBlockMotorRearR.isChecked() == False:
                SpeedControlDev11(Position)
            print(Position)

    elif TimerState == 3:
        # STOP – zero speed immediately
        Position = 0
        ui.progressBarLoad.setValue(Position)
        if ui.checkBoxBlockMotorFrontL.isChecked() == False:
            SpeedControlDev8(Position)
        if ui.checkBoxBlockMotorFrontR.isChecked() == False:
            SpeedControlDev9(Position)
        if ui.checkBoxBlockMotorRearL.isChecked() == False:
            SpeedControlDev10(Position)
        if ui.checkBoxBlockMotorRearR.isChecked() == False:
            SpeedControlDev11(Position)
        print(Position)


# ---------------------------------------------------------------------------
# UI wiring
# ---------------------------------------------------------------------------

ports = QSerialPortInfo.availablePorts()
for i in ports:
    PortList.append(i.portName())
ui.PortNumberCombo.addItems(PortList)

ui.PortOpenButton.clicked.connect(ClickPortOpenButton)

# Manipulator
ui.pushButtonManipulatorUp.pressed.connect(pressedpushButtonManipulatorUp)
ui.pushButtonManipulatorUp.released.connect(releasedpushButtonManipulatorUp)
ui.pushButtonManipulatorDown.pressed.connect(pressedpushButtonManipulatorDown)
ui.pushButtonManipulatorDown.released.connect(releasedpushButtonManipulatorDown)
ui.pushButtonManipulatorLeft.pressed.connect(pressedpushButtonManipulatorLeft)
ui.pushButtonManipulatorLeft.released.connect(releasedpushButtonManipulatorLeft)
ui.pushButtonManipulatorRight.pressed.connect(pressedpushButtonManipulatorRight)
ui.pushButtonManipulatorRight.released.connect(releasedpushButtonManipulatorRight)

# Ladle
ui.pushButtonLadleUp.clicked.connect(clickedpushButtonLadleUp)
ui.pushButtonLadleDown.pressed.connect(pressedpushButtonLadleDown)
ui.pushButtonLadleDown.released.connect(releasedpushButtonLadleDown)

# Frame
ui.pushButtonFrameDown.clicked.connect(clickedpushButtonFrameDown)
ui.pushButtonFrameUp.pressed.connect(pressedpushButtonFrameUp)
ui.pushButtonFrameUp.released.connect(releasedpushButtonFrameUp)

# Bunker
ui.pushButtonBunkerDown.clicked.connect(clickedpushButtonBunkerDown)
ui.pushButtonBunkerUp.pressed.connect(pressedpushButtonBunkerUp)
ui.pushButtonBunkerUp.released.connect(releasedpushButtonBunkerUp)

# Flaps
ui.pushButtonFlapsDown.clicked.connect(clickedpushButtonFlapsDown)
ui.pushButtonFlapsUp.pressed.connect(pressedpushButtonFlapsUp)
ui.pushButtonFlapsUp.released.connect(releasedpushButtonFlapsUp)

# Separator
ui.pushButtonSeparatorUp.clicked.connect(clickedpushButtonSeparatorUp)
ui.pushButtonSeparatorDown.clicked.connect(clickedpushButtonSeparatorDown)
ui.pushButtonSeparatorStop.clicked.connect(clickedpushButtonSeparatorStop)

# Steering
ui.pushButtonSteeringLeft.pressed.connect(pressedpushButtonSteeringLeft)
ui.pushButtonSteeringLeft.released.connect(releasedpushButtonSteeringLeft)
ui.pushButtonSteeringRight.pressed.connect(pressedpushButtonSteeringRight)
ui.pushButtonSteeringRight.released.connect(releasedpushButtonSteeringRight)

# Reverser
ui.pushButtonReverserUp.clicked.connect(clickedpushButtonReverserUp)
ui.pushButtonReverserDown.clicked.connect(clickedpushButtonReverserDown)

# Position / speed ramp
ui.pushButtonPositionSet.pressed.connect(pressedpushButtonPositionSet)
ui.pushButtonPositionSet.released.connect(releasedpushButtonPositionSet)
ui.pushButtonPositionReset.pressed.connect(pressedpushButtonPositionReset)
ui.pushButtonPositionReset.released.connect(releasedpushButtonPositionReset)

# Drive stop
ui.pushButtonDriveStop.pressed.connect(pressedpushButtonDriveStop)
ui.pushButtonDriveStop.released.connect(releasedpushButtonDriveStop)

# Timer
timer.timeout.connect(TimeoutTimer)

# ---------------------------------------------------------------------------
# LED setup – Square shape, Green=OK, Red=Error, Grey=Off
# ---------------------------------------------------------------------------

LedStateManipulator.setOnColour(LedStateManipulator.Green)
LedStateManipulator.setOffColour(LedStateManipulator.Grey)
LedStateManipulator.setShape(LedStateManipulator.Square)
LedStateManipulator.setValue(False)

LedStateLadle.setOnColour(LedStateLadle.Green)
LedStateLadle.setOffColour(LedStateLadle.Grey)
LedStateLadle.setShape(LedStateLadle.Square)
LedStateLadle.setValue(False)

LedStateFrame.setOnColour(LedStateFrame.Green)
LedStateFrame.setOffColour(LedStateFrame.Grey)
LedStateFrame.setShape(LedStateFrame.Square)
LedStateFrame.setValue(False)

LedStateBunker.setOnColour(LedStateBunker.Green)
LedStateBunker.setOffColour(LedStateBunker.Grey)
LedStateBunker.setShape(LedStateBunker.Square)
LedStateBunker.setValue(False)

LedStateFlaps.setOnColour(LedStateFlaps.Green)
LedStateFlaps.setOffColour(LedStateFlaps.Grey)
LedStateFlaps.setShape(LedStateFlaps.Square)
LedStateFlaps.setValue(False)

LedStateSeparator.setOnColour(LedStateSeparator.Green)
LedStateSeparator.setOffColour(LedStateSeparator.Grey)
LedStateSeparator.setShape(LedStateSeparator.Square)
LedStateSeparator.setValue(False)

LedStateWheel.setOnColour(LedStateWheel.Green)
LedStateWheel.setOffColour(LedStateWheel.Grey)
LedStateWheel.setShape(LedStateWheel.Square)
LedStateWheel.setValue(False)

LedStateMotorFrontL.setOnColour(LedStateMotorFrontL.Green)
LedStateMotorFrontL.setOffColour(LedStateMotorFrontL.Grey)
LedStateMotorFrontL.setShape(LedStateMotorFrontL.Square)
LedStateMotorFrontL.setValue(False)

LedStateMotorFrontR.setOnColour(LedStateMotorFrontR.Green)
LedStateMotorFrontR.setOffColour(LedStateMotorFrontR.Grey)
LedStateMotorFrontR.setShape(LedStateMotorFrontR.Square)
LedStateMotorFrontR.setValue(False)

LedStateMotorRearL.setOnColour(LedStateMotorRearL.Green)
LedStateMotorRearL.setOffColour(LedStateMotorRearL.Grey)
LedStateMotorRearL.setShape(LedStateMotorRearL.Square)
LedStateMotorRearL.setValue(False)

LedStateMotorRearR.setOnColour(LedStateMotorRearR.Green)
LedStateMotorRearR.setOffColour(LedStateMotorRearR.Grey)
LedStateMotorRearR.setShape(LedStateMotorRearR.Square)
LedStateMotorRearR.setValue(False)

# Add LED widgets to layout slots
ui.horizontalLayout_2.addWidget(LedStateManipulator)
ui.horizontalLayout_3.addWidget(LedStateLadle)
ui.horizontalLayout_4.addWidget(LedStateFrame)
ui.horizontalLayout_5.addWidget(LedStateBunker)
ui.horizontalLayout_6.addWidget(LedStateFlaps)
ui.horizontalLayout_7.addWidget(LedStateSeparator)
ui.horizontalLayout_17.addWidget(LedStateWheel)
ui.horizontalLayout_12.addWidget(LedStateMotorFrontL)
ui.horizontalLayout_13.addWidget(LedStateMotorFrontR)
ui.horizontalLayout_15.addWidget(LedStateMotorRearL)
ui.horizontalLayout_16.addWidget(LedStateMotorRearR)

# Disable all control groups until connected
ui.groupBoxControlManipulator.setEnabled(False)
ui.groupBoxControlLadle.setEnabled(False)
ui.groupBoxControlFrame.setEnabled(False)
ui.groupBoxControlBunker.setEnabled(False)
ui.groupBoxControlFlaps.setEnabled(False)
ui.groupBoxControlSeparator.setEnabled(False)
ui.groupBoxControlSteering.setEnabled(False)
ui.groupBoxControlDrive.setEnabled(False)

# Start keyboard listener (pynput)
listener = keyboard.Listener(on_press=on_press, on_release=on_release)
listener.start()

ui.show()
app.exec()
