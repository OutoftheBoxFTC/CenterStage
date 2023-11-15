package org.firstinspires.ftc.teamcode.hardware.devices

interface KDevice {
    fun writeData()

    fun readCurrent(): Double
}
