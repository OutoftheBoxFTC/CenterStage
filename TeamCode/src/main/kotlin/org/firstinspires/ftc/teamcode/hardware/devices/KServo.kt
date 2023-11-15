package org.firstinspires.ftc.teamcode.hardware.devices

import com.qualcomm.robotcore.hardware.Servo

class KServo(private val servo: Servo) : KDevice, Servo by servo {
    private var position: Double? = null

    override fun setPosition(position: Double) { this.position = position }

    override fun writeData() {
        position?.let {
            servo.position = it
            position = null
        }
    }

    override fun readCurrent() = 0.0
}