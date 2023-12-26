package org.firstinspires.ftc.teamcode.hardware.devices

import com.qualcomm.robotcore.hardware.CRServo
import kotlin.math.abs

/**
 * A wrapper for [CRServo] that implements [KDevice] with power tolerance optimization.
 */
class KCRServo(private val servo: CRServo): KDevice, CRServo by servo {
    var powerTolerance = 1.0 / 8192

    private var power: Double? = null

    override fun setPower(power: Double) { this.power = power }
    override fun writeData() {
        power?.let {
            if (abs(it - servo.power) > powerTolerance) servo.power = it
            power = null
        }
    }

    override fun readCurrent() = 0.0
}