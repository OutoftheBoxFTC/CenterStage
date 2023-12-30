package org.firstinspires.ftc.teamcode.hardware.devices

import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.util.G
import kotlin.math.abs

/**
 * A wrapper for [Servo] that implements [KDevice] with position tolerance optimization.
 */
class KServo(private val servo: Servo) : KDevice, Servo by servo {
    var positionTolerance = 1.0 / 8192

    private var position: Double? = null

    override fun setPosition(position: Double) { this.position = position }

    override fun writeData() {
        position?.let {
            if (G.isInit || abs(it - servo.position) > positionTolerance) servo.position = it
            position = null
        }
    }

    override fun readCurrent() = 0.0
}