package org.firstinspires.ftc.teamcode.util

import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit

fun DcMotorEx.cloneMotor() = object : DcMotorEx by this {
    private var direction: Direction = Direction.FORWARD

    private val Direction.multiplier get() = if (this == Direction.FORWARD) 1 else -1
    private val multiplier get() = direction.multiplier * this@cloneMotor.direction.multiplier

    override fun getDirection() = direction
    override fun setDirection(direction: Direction) { this.direction = direction }

    override fun getPower() = multiplier * this@cloneMotor.power
    override fun setPower(power: Double) { this@cloneMotor.power = multiplier * power }

    override fun getCurrentPosition() = this@cloneMotor.currentPosition * multiplier
    override fun getTargetPosition() = this@cloneMotor.targetPosition * multiplier
    override fun setTargetPosition(position: Int) { this@cloneMotor.targetPosition = position }

    override fun getVelocity() = this@cloneMotor.velocity * multiplier
    override fun getVelocity(unit: AngleUnit) = this@cloneMotor.getVelocity(unit) * multiplier

    override fun setVelocity(angularRate: Double) {
        this@cloneMotor.velocity = angularRate * multiplier
    }

    override fun setVelocity(angularRate: Double, unit: AngleUnit?) {
        this@cloneMotor.setVelocity(angularRate * multiplier, unit)
    }
}