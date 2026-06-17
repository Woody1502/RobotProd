from PyQt5 import QtWidgets, uic
from PyQt5.QtSerialPort import QSerialPort, QSerialPortInfo
from PyQt5.QtCore import QIODevice, QTimer
from QLed import QLed
import sys
import os
import numpy as np
import minimalmodbus

BaudRateList = ['1200','2400','4800', '9600','19200','38400','57600','115200']
PortList = []
CounterPacketRx = 0
CounterPacketTx = 0
Enable_Motor = False
Direction_Motor = False
Speed = 0
Error_MB = False
Start_Stop_Flag = True
Direction_Flag = True
Speed_Value_Flag = True
Rev = False

app = QtWidgets.QApplication([])
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
ui = uic.loadUi(os.path.join(BASE_DIR, 'Interf.ui'))
timer = QTimer()
LedRx = QLed(); LedTx = QLed();

def ClickPortOpenButton():
    global CounterPacketRx, CounterPacketTx, Motor_Driver
    if ui.PortOpenButton.text() == "Подключить":
        Motor_Driver = minimalmodbus.Instrument(ui.PortNumberCombo.currentText(), 1)
        Motor_Driver.serial.baudrate = int(ui.PortRateCombo.currentText())
        Motor_Driver.serial.timeout = 0.2
        Motor_Driver.address = int(ui.AddresMBSpinBox.value())
        Motor_Driver.mode = minimalmodbus.MODE_RTU

        ui.PortOpenButton.setText("Отключить")
        ui.groupBox.setEnabled(False)
        ui.groupBox_2.setEnabled(True)
        ui.groupBox_3.setEnabled(True)
        timer.start(100)
    elif ui.PortOpenButton.text() == "Отключить":
        Motor_Driver.serial.close()
        timer.stop()
        CounterPacketRx = 0
        CounterPacketTx = 0
        ui.PortOpenButton.setText("Подключить")
        ui.groupBox.setEnabled(True)
        ui.groupBox_2.setEnabled(False)
        ui.groupBox_3.setEnabled(False)

def ClickStartButton():
    global Enable_Motor, Start_Stop_Flag
    if ui.StartButton.text() == "ПУСК":
        ui.StartButton.setText("СТОП")
        Enable_Motor = True
        Start_Stop_Flag = True
    elif ui.StartButton.text() == "СТОП":
        ui.StartButton.setText("ПУСК")
        Enable_Motor = False
        Start_Stop_Flag = True

def ClickRevButton():
    global Direction_Motor, Direction_Flag, Rev
    if Rev == False:
        Rev = True
        Direction_Motor = True
        Direction_Flag = True
    elif Rev == True:
        Rev = False
        Direction_Motor = False
        Direction_Flag = True

def Slider_Speed_Value (value):
    global Speed, Speed_Value_Flag
    ui.SpeedlcdNumber.display(value)
    Speed = value
    Speed_Value_Flag = True

def Read_Write_Data_MB ():
    global CounterPacketTx, CounterPacketRx, Error_MB, Start_Stop_Flag, Direction_Flag, Speed_Value_Flag
    try:
        CounterPacketTx += 1
        ui.labelCountTx.setText('TX:' + str(CounterPacketTx))
        LedRx.value = False
        LedTx.value = True
        if Start_Stop_Flag == True:
            Start_Stop_Flag = False
            Motor_Driver.write_bit(0, Enable_Motor, 5)
        if Direction_Flag == True:
            Direction_Flag = False
            Motor_Driver.write_bit(1, Direction_Motor, 5)
        if Speed_Value_Flag == True:
            Speed_Value_Flag = False
            Motor_Driver.write_register(0, Speed, 0, 6, False)
        Frequency = Motor_Driver.read_register(0,0,4,False)
        ui.FreqlcdNumber.display(Frequency)
        LedRx.value = True
        LedTx.value = False
        Error_MB = False
    except minimalmodbus.ModbusException:
        Error_MB = True

    if Error_MB == False:
        CounterPacketRx += 1
        ui.labelCountRx.setText('RX:' + str(CounterPacketRx))

ui.PortRateCombo.addItems(BaudRateList)
ports = QSerialPortInfo.availablePorts()
for i in ports:
    PortList.append(i.portName())
ui.PortNumberCombo.addItems(PortList)

ui.PortOpenButton.clicked.connect(ClickPortOpenButton)
ui.StartButton.clicked.connect(ClickStartButton)
ui.RevButton.clicked.connect(ClickRevButton)
ui.Slider_Speed.valueChanged.connect(Slider_Speed_Value)
timer.timeout.connect(Read_Write_Data_MB)
ui.groupBox_2.setEnabled(False)
ui.groupBox_3.setEnabled(False)

LedRx.setOnColour(LedRx.Green); LedRx.setOffColour(LedRx.Grey); LedRx.setShape(LedRx.Round); LedRx.setValue(False)
LedTx.setOnColour(LedTx.Red); LedTx.setOffColour(LedTx.Grey); LedTx.setShape(LedTx.Round); LedTx.setValue(False)

ui.verticalLayout_5.addWidget(LedRx)
ui.verticalLayout_5.addWidget(LedTx)

ui.show()
app.exec()