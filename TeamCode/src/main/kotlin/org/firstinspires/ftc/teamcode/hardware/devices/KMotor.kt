package org.firstinspires.ftc.teamcode.hardware.devices

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.PIDFCoefficients
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import org.firstinspires.ftc.teamcode.util.G
import kotlin.math.abs

/**
 * A wrapper for [DcMotorEx] that implements [KDevice] with power tolerance optimization.
 */
class KMotor(private val motor: DcMotorEx) : KDevice, DcMotorEx by motor {
    var powerTolerance = 1.0 / 8192

    private var motorType: MotorConfigurationType? = null

    private var power: Double? = null

    private var zeroPowerBehavior: DcMotor.ZeroPowerBehavior? = null

    private var targetPosition: Int? = null

    private var runMode: DcMotor.RunMode? = null

    private var enableMotor: Boolean? = null

    private var nextMotorVelocity: Double? = null
    private var nextMotorVelocityUnit: AngleUnit? = null

    private val runmodePIDFCoefs = mutableMapOf<DcMotor.RunMode, PIDFCoefficients>()

    private var positionPIDFCoefs: PIDFCoefficients? = null
    private var velocityPIDFCoefs: PIDFCoefficients? = null

    private var lastCurrent = 0.0

    private var currentAlert: Double? = null

    private var isOverCurrent = false

    override fun setMotorType(motorType: MotorConfigurationType) { this.motorType = motorType }

    override fun setPower(power: Double) { this.power = power }

    override fun setZeroPowerBehavior(zeroPowerBehavior: DcMotor.ZeroPowerBehavior) {
        this.zeroPowerBehavior = zeroPowerBehavior
    }

    override fun setTargetPosition(position: Int) { targetPosition = position }

    override fun setMode(mode: DcMotor.RunMode) { runMode = mode }

    override fun setMotorEnable() { enableMotor = true }
    override fun setMotorDisable() { enableMotor = false }

    override fun setVelocity(angularRate: Double) { nextMotorVelocity = angularRate }
    override fun setVelocity(angularRate: Double, unit: AngleUnit) {
        nextMotorVelocity = angularRate
        nextMotorVelocityUnit = unit
    }

    override fun setPIDFCoefficients(mode: DcMotor.RunMode, pidfCoefficients: PIDFCoefficients) {
        runmodePIDFCoefs[mode] = pidfCoefficients
    }

    override fun setVelocityPIDFCoefficients(p: Double, i: Double, d: Double, f: Double) {
        velocityPIDFCoefs = PIDFCoefficients(p, i, d, f)
    }

    override fun setPositionPIDFCoefficients(p: Double) {
        positionPIDFCoefs = PIDFCoefficients(p, 0.0, 0.0, 0.0)
    }

    override fun getCurrent(unit: CurrentUnit) = unit.toAmps(lastCurrent)

    override fun setCurrentAlert(current: Double, unit: CurrentUnit) {
        currentAlert = unit.toAmps(current)
    }

    override fun isOverCurrent() = isOverCurrent

    override fun writeData() {
        motorType?.let {
            motor.motorType = it
            motorType = null
        }

        power?.let {
            if (G.isInit || abs(it - motor.power) > powerTolerance) motor.power = it
            power = null
        }

        zeroPowerBehavior?.let {
            motor.zeroPowerBehavior = it
            zeroPowerBehavior = null
        }

        targetPosition?.let {
            motor.targetPosition = it
            targetPosition = null
        }

        runMode?.let {
            motor.mode = runMode
            runMode = null
        }

        enableMotor?.let {
            if (it) motor.setMotorEnable() else motor.setMotorDisable()
            enableMotor = null
        }

        nextMotorVelocity?.let { angRate ->
            nextMotorVelocityUnit?.let { motor.setVelocity(angRate, it) }
                ?: motor.setVelocity(angRate)

            nextMotorVelocity = null
            nextMotorVelocityUnit = null
        }

        runmodePIDFCoefs.forEach { motor.setPIDFCoefficients(it.key, it.value) }
        runmodePIDFCoefs.clear()

        positionPIDFCoefs?.let {
            motor.setPositionPIDFCoefficients(it.p)
            positionPIDFCoefs = null
        }

        velocityPIDFCoefs?.let {
            motor.setVelocityPIDFCoefficients(it.p, it.i, it.d, it.f)
            velocityPIDFCoefs = null
        }

        currentAlert?.let {
            motor.setCurrentAlert(it, CurrentUnit.AMPS)
            currentAlert = null
        }
    }

    override fun readCurrent() = motor.getCurrent(CurrentUnit.AMPS).also {
        lastCurrent = it
        isOverCurrent = motor.isOverCurrent
    }

}