@file:Suppress("MemberVisibilityCanBePrivate")

package org.firstinspires.ftc.teamcode.hardware

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import kotlin.math.abs

class KMotor {
    var powerTolerance = 1.0 / 8192

    private var lastPower = 0.0
    var power = lastPower
    var current = 0.0
        private set

    var currentPosition = 0
        private set

    var direction = DcMotorSimple.Direction.FORWARD
    var zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

    var enableWrites = fun() = true

    fun applyChanges(motor: DcMotorEx) {
        if (enableWrites() && abs(lastPower - power) > powerTolerance) {
            motor.power = power
            lastPower = power
        }

        current = motor.getCurrent(CurrentUnit.AMPS)
        currentPosition = motor.currentPosition

        motor.direction = direction
        motor.zeroPowerBehavior = zeroPowerBehavior
    }
}

class KServo {
    var positionTolerance: Double = 1.0 / 1024

    private var lastPosition = 0.0
    var position = lastPosition

    fun applyChanges(servo: Servo)  {
        if (abs(lastPosition - position) > positionTolerance) {
            servo.position = position
            lastPosition = position
        }
    }
}
