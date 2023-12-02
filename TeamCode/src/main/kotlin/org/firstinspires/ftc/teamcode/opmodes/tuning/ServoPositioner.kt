package org.firstinspires.ftc.teamcode.opmodes.tuning

import arrow.core.merge
import arrow.fx.coroutines.raceN
import com.acmerobotics.dashboard.config.Config
import com.outoftheboxrobotics.suspendftc.suspendUntil
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.util.ElapsedTime
import kotlinx.coroutines.cancelAndJoin
import kotlinx.coroutines.coroutineScope
import kotlinx.coroutines.launch
import org.firstinspires.ftc.teamcode.actions.hardware.IntakeTiltPosition
import org.firstinspires.ftc.teamcode.actions.hardware.setArmPosition
import org.firstinspires.ftc.teamcode.actions.hardware.setTiltPosition
import org.firstinspires.ftc.teamcode.actions.hardware.setTwistPosition
import org.firstinspires.ftc.teamcode.opmodes.RobotOpMode
import org.firstinspires.ftc.teamcode.runStateMachine
import org.firstinspires.ftc.teamcode.util.FS
import org.firstinspires.ftc.teamcode.util.mainLoop
import org.firstinspires.ftc.teamcode.util.set

@TeleOp
@Config
class ServoPositioner : RobotOpMode() {
    companion object {
        @JvmField var incrementSpeed = 0.2
    }

    private var armPos = 0.0
    private var twistPos = 0.0
    private var tiltPos = 0.0

    private var currentSystem = "Arm"
    private var currentPos = armPos

    private suspend fun nextToggledState(prev: FS, next: FS): FS = coroutineScope {
        raceN(
            coroutineContext,
            {
                suspendUntil { !gamepad1.x }
                suspendUntil { gamepad1.x }
                prev
            },
            {
                suspendUntil { !gamepad1.b }
                suspendUntil { gamepad1.b }
                next
            }
        ).merge()
    }

    private val armAdjust: FS = FS {
        currentSystem = "Arm"
        currentPos = armPos

        val job = launch {
            mainLoop {
                armPos = currentPos
            }
        }

        nextToggledState(tiltAdjust, twistAdjust).also { job.cancelAndJoin() }
    }

    private val twistAdjust: FS = FS {
        currentSystem = "Twist"
        currentPos = twistPos

        val job = launch {
            mainLoop {
                twistPos = currentPos
            }
        }

        nextToggledState(armAdjust, tiltAdjust).also { job.cancelAndJoin() }
    }

    private val tiltAdjust: FS = FS {
        currentSystem = "Tilt"
        currentPos = tiltPos

        val job = launch {
            mainLoop {
                tiltPos = currentPos
            }
        }

        nextToggledState(twistAdjust, armAdjust).also { job.cancelAndJoin() }
    }

    override suspend fun runSuspendOpMode() = coroutineScope {
        armPos = 0.5
        twistPos = 0.5
        tiltPos = IntakeTiltPosition.HIGH.pos

        suspendUntilStart()

        val timer = ElapsedTime()

        launch {
            runStateMachine(armAdjust)
        }

        mainLoop {
            setArmPosition(armPos)
            setTwistPosition(twistPos)
            setTiltPosition(tiltPos)

            telemetry["Arm"] = armPos
            telemetry["Twist"] = twistPos
            telemetry["Tilt"] = tiltPos

            telemetry["Current Servo"] = currentSystem

            val dt = timer.seconds()
            timer.reset()

            currentPos = when {
                gamepad1.left_bumper -> currentPos + incrementSpeed * dt
                gamepad1.right_bumper -> currentPos - incrementSpeed * dt
                else -> currentPos
            }.coerceIn(0.0..1.0)
        }
    }
}