package org.firstinspires.ftc.teamcode.opmodes.testing

import com.acmerobotics.dashboard.config.Config
import com.outoftheboxrobotics.suspendftc.loopYieldWhile
import com.outoftheboxrobotics.suspendftc.suspendFor
import com.outoftheboxrobotics.suspendftc.suspendUntil
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.util.ElapsedTime
import kotlinx.coroutines.cancelAndJoin
import kotlinx.coroutines.coroutineScope
import kotlinx.coroutines.launch
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import org.firstinspires.ftc.teamcode.actions.hardware.IntakeTiltPosition
import org.firstinspires.ftc.teamcode.opmodes.RobotOpMode
import org.firstinspires.ftc.teamcode.runStateMachine
import org.firstinspires.ftc.teamcode.util.FS
import org.firstinspires.ftc.teamcode.util.G
import org.firstinspires.ftc.teamcode.util.mainLoop
import org.firstinspires.ftc.teamcode.util.set

@TeleOp
@Config
@Disabled
class IntakeTest : RobotOpMode(
    runMultiThreaded = true,
    imuRunMode = ImuRunMode.THREADED
) {
    companion object {
        @JvmField var maxRollerCurrent = 6.0
        @JvmField var manualIntakePower = -0.3
    }
    // Intake roller state machine
    private val rollerIntakeState: FS = FS {
        val job = launch {
            while (true) {
                G.ehub.readCurrents()

                if (G.ehub.intakeRoller.getCurrent(CurrentUnit.AMPS) > maxRollerCurrent) {
                    G.ehub.intakeRoller.power = 1.0
                    suspendFor(20)
                    G.ehub.intakeRoller.power = -1.0
                } else G.ehub.intakeRoller.power = -1.0

                suspendFor(200)
            }
        }

        suspendUntil { !gamepad1.a }

        job.cancelAndJoin()

        rollerDefaultState
    }

    private val rollerDefaultState: FS = FS {
        loopYieldWhile({ !gamepad1.a }) {
            G.ehub.intakeRoller.power =
                if (gamepad1.x) manualIntakePower
                else gamepad1.left_stick_y.toDouble()
        }

        rollerIntakeState
    }

    override suspend fun runSuspendOpMode() = coroutineScope {
        suspendUntilStart()

        val timer = ElapsedTime()
        var servoPos = IntakeTiltPosition.LOW.pos

        val increment = 0.05

        launch { runStateMachine(rollerDefaultState) }

        mainLoop {
            val dt = timer.seconds()
            timer.reset()

            when {
                gamepad1.right_bumper -> servoPos += increment * dt
                gamepad1.left_bumper -> servoPos -= increment * dt
            }

            G.chub.intakeTilt.position = servoPos
            telemetry["servo position"] = servoPos
        }
    }
}