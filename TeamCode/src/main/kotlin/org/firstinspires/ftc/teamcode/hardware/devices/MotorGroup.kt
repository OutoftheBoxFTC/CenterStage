package org.firstinspires.ftc.teamcode.hardware.devices

import arrow.core.Nel
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorController
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareDevice
import com.qualcomm.robotcore.hardware.PIDCoefficients
import com.qualcomm.robotcore.hardware.PIDFCoefficients
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit

/**
 * A group of motors that act as one.
 */
class MotorGroup(private val motors: Nel<DcMotorEx>) : DcMotorEx {
    private var direction = DcMotorSimple.Direction.FORWARD

    private val multiplier get() = if (direction == DcMotorSimple.Direction.FORWARD) 1 else -1

    override fun getManufacturer(): HardwareDevice.Manufacturer = motors.head.manufacturer
    override fun getDeviceName() = motors.map { it.deviceName }.joinToString()
    override fun getConnectionInfo() = motors.map { it.connectionInfo }.joinToString()
    override fun getVersion() = motors.head.version

    override fun resetDeviceConfigurationForOpMode() = motors.forEach { it.resetDeviceConfigurationForOpMode() }
    override fun close() = motors.forEach { it.close() }

    override fun setDirection(direction: DcMotorSimple.Direction) { this.direction = direction }
    override fun getDirection() = direction

    override fun setPower(power: Double) = motors.forEach { it.power = power * multiplier }
    override fun getPower() = motors.head.power * multiplier

    override fun getMotorType(): MotorConfigurationType = motors.head.motorType
    override fun setMotorType(motorType: MotorConfigurationType) =
        motors.forEach { it.motorType = motorType }

    override fun getController(): DcMotorController = motors.head.controller

    override fun getPortNumber() = motors.head.portNumber

    override fun setZeroPowerBehavior(zeroPowerBehavior: DcMotor.ZeroPowerBehavior) =
        motors.forEach { it.zeroPowerBehavior = zeroPowerBehavior }

    override fun getZeroPowerBehavior(): DcMotor.ZeroPowerBehavior = motors.head.zeroPowerBehavior

    @Suppress("OVERRIDE_DEPRECATION", "DEPRECATION")
    override fun setPowerFloat() = motors.forEach { it.setPowerFloat() }
    override fun getPowerFloat() = motors.head.powerFloat

    override fun setTargetPosition(position: Int) =
        motors.forEach { it.targetPosition = position * multiplier }
    override fun getTargetPosition() = motors.head.targetPosition * multiplier

    override fun isBusy() = motors.any { it.isBusy }

    override fun getCurrentPosition() = motors.head.currentPosition * multiplier

    override fun setMode(mode: DcMotor.RunMode) = motors.forEach { it.mode = mode }
    override fun getMode(): DcMotor.RunMode = motors.head.mode

    override fun setMotorEnable() = motors.forEach { it.setMotorEnable() }
    override fun setMotorDisable() = motors.forEach { it.setMotorDisable() }

    override fun isMotorEnabled() = motors.any { it.isMotorEnabled }

    override fun setVelocity(angularRate: Double) =
        motors.forEach { it.velocity = angularRate * multiplier }

    override fun setVelocity(angularRate: Double, unit: AngleUnit) =
        motors.forEach { it.setVelocity(angularRate * multiplier, unit) }

    override fun getVelocity() = motors.head.velocity * multiplier

    override fun getVelocity(unit: AngleUnit) = motors.head.getVelocity(unit) * multiplier

    @Suppress("OVERRIDE_DEPRECATION", "DEPRECATION")
    override fun setPIDCoefficients(mode: DcMotor.RunMode, pidCoefficients: PIDCoefficients) =
        motors.forEach { it.setPIDCoefficients(mode, pidCoefficients) }

    override fun setPIDFCoefficients(mode: DcMotor.RunMode, pidfCoefficients: PIDFCoefficients) =
        motors.forEach { it.setPIDFCoefficients(mode, pidfCoefficients) }

    override fun setVelocityPIDFCoefficients(p: Double, i: Double, d: Double, f: Double) =
        motors.forEach { it.setVelocityPIDFCoefficients(p, i, d, f) }

    override fun setPositionPIDFCoefficients(p: Double) =
        motors.forEach { it.setPositionPIDFCoefficients(p) }

    @Suppress("OVERRIDE_DEPRECATION", "DEPRECATION")
    override fun getPIDCoefficients(mode: DcMotor.RunMode): PIDCoefficients =
        motors.head.getPIDCoefficients(mode)

    override fun getPIDFCoefficients(mode: DcMotor.RunMode): PIDFCoefficients =
        motors.head.getPIDFCoefficients(mode)

    override fun setTargetPositionTolerance(tolerance: Int) =
        motors.forEach { it.targetPositionTolerance = tolerance }

    override fun getTargetPositionTolerance() = motors.head.targetPositionTolerance

    override fun getCurrent(unit: CurrentUnit) = motors.map { it.getCurrent(unit) }.sum()

    override fun getCurrentAlert(unit: CurrentUnit) = motors.head.getCurrentAlert(unit)

    override fun setCurrentAlert(current: Double, unit: CurrentUnit) =
        motors.forEach { it.setCurrentAlert(current, unit) }

    override fun isOverCurrent() = motors.any { it.isOverCurrent }
}