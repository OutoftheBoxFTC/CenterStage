package org.firstinspires.ftc.teamcode.opmodes

import arrow.core.raise.nullable
import com.acmerobotics.dashboard.config.Config
import com.outoftheboxrobotics.suspendftc.suspendUntil
import com.outoftheboxrobotics.suspendftc.yieldLooper
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.util.ElapsedTime
import kotlinx.coroutines.coroutineScope
import kotlinx.coroutines.launch
import org.apache.commons.math3.stat.regression.SimpleRegression
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import org.firstinspires.ftc.teamcode.util.G
import org.firstinspires.ftc.teamcode.util.Interplut
import org.firstinspires.ftc.teamcode.util.cancellationScope
import org.firstinspires.ftc.teamcode.util.lerp
import org.firstinspires.ftc.teamcode.util.mainLoop
import org.firstinspires.ftc.teamcode.util.set
import org.firstinspires.ftc.teamcode.util.suspendUntilRisingEdge

@TeleOp
@Config
class GearRatioOptimizer : RobotOpMode() {
    companion object {
        @JvmField var ticksPerRot = 28
        @JvmField var currentGear = 3.7
    }

    data class MotorSpec(val stallTorque: Double, val stallCurrent: Double, val maxRpm: Double) {
        private val freeCurrent = 0.25

        fun torque(rpm: Double) = lerp(stallTorque, 0.0, rpm / maxRpm)
        fun current(rpm: Double) = lerp(stallCurrent, freeCurrent, rpm / maxRpm)
    }

    private val base12VSpec = MotorSpec(0.19, 11.0, 5900.0)

    private val stallTorque = buildMap {
        put(12.0, 0.19)
        put(10.0, 0.16)
        put(8.0, 0.13)
        put(6.0, 0.092)
        put(4.0, 0.06)
    }.let { Interplut(it) }

    private val stallCurrent = buildMap {
        put(12.0, 11.0)
        put(10.0, 9.0)
        put(8.0, 7.2)
        put(6.0, 5.3)
        put(4.0, 3.5)
    }.let { Interplut(it) }

    private val maxRpm = buildMap {
        put(12.0, 5900.0)
        put(10.0, 5030.0)
        put(8.0, 3940.0)
        put(6.0, 2920.0)
        put(4.0, 1900.0)
    }.let { Interplut(it) }

    fun getMotorSpec(v: Double) = nullable {
        MotorSpec(
            stallTorque[v].bind(),
            stallCurrent[v].bind(),
            maxRpm[v].bind(),
        )
    } ?: base12VSpec.run {
        MotorSpec(
            stallTorque * v / 12.0,
            stallCurrent * v / 12.0,
            maxRpm * v / 12.0
        )
    }

    override suspend fun runSuspendOpMode() = coroutineScope {
        val motor: DcMotorEx = G.ehub.outtakeLift
        val targetPower = 1.0

        launch {
            mainLoop {
                G.chub.readCurrents()
                G.ehub.readCurrents()
            }
        }

        suspendUntilStart()

        suspendUntilRisingEdge { gamepad1.right_bumper }

        val mainTimer = ElapsedTime()

        val torqueReg = SimpleRegression()
        val currentReg = SimpleRegression()

        var totalRotations = 0.0

        motor.power = targetPower

        cancellationScope {
            launch {
                val timer = ElapsedTime()
                var lastRpm = 60 * motor.velocity / ticksPerRot

                yieldLooper()

                mainLoop {
                    val dt = timer.seconds()
                    timer.reset()

                    val rpm = 60 * motor.velocity / ticksPerRot
                    val v = G.chub.voltageSensor.voltage
                    val a = motor.getCurrent(CurrentUnit.AMPS)

                    val spec = getMotorSpec(v)

                    val t = spec.torque((rpm + lastRpm) / 2) * currentGear
                    val accel = (rpm - lastRpm) / dt

                    torqueReg.addData(t, accel)

                    currentReg.addData(a, v)

                    totalRotations += rpm * dt / currentGear

                    telemetry["rpm"] = rpm
                    telemetry["current"] = a
                    telemetry["expected current"] = spec.current(rpm)
                    telemetry["voltage"] = v
                    telemetry["rotations"] = totalRotations
                    telemetry["torque"] = t
                    telemetry["accel"] = accel

                    lastRpm = rpm
                }
            }

            suspendUntil { !gamepad1.right_bumper }
        }

        motor.power = 0.0

        mainLoop {
            telemetry["r^2"] = torqueReg.rSquare
            telemetry["slope"] = torqueReg.slope
            telemetry["intercept"] = torqueReg.intercept
        }
    }
}