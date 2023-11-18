package org.firstinspires.ftc.teamcode.opmodes.tuning

import arrow.core.nel
import arrow.core.some
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.outoftheboxrobotics.suspendftc.loopYieldWhile
import com.outoftheboxrobotics.suspendftc.suspendUntil
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import kotlinx.coroutines.coroutineScope
import kotlinx.coroutines.launch
import org.firstinspires.ftc.teamcode.actions.controllers.PidCoefs
import org.firstinspires.ftc.teamcode.actions.controllers.runPosePidController
import org.firstinspires.ftc.teamcode.actions.hardware.setAdjustedDrivePowers
import org.firstinspires.ftc.teamcode.actions.hardware.setDrivePowers
import org.firstinspires.ftc.teamcode.command.Subsystem
import org.firstinspires.ftc.teamcode.hardware.devices.ThreadedImuHandler
import org.firstinspires.ftc.teamcode.opmodes.RobotOpMode
import org.firstinspires.ftc.teamcode.statemachine.runStateMachine
import org.firstinspires.ftc.teamcode.util.C
import org.firstinspires.ftc.teamcode.util.FS
import org.firstinspires.ftc.teamcode.util.G
import org.firstinspires.ftc.teamcode.util.launchCommand
import org.firstinspires.ftc.teamcode.util.set

@TeleOp
@Config
class FixpointPidTuner : RobotOpMode(
    runMultiThreaded = true,
    imuHandler = ThreadedImuHandler().some()
) {
    companion object {
        @JvmField var target_x = 0.0
        @JvmField var target_y = 0.0
        @JvmField var target_heading = 0.0

        @JvmField var heading_kP = 3.2
        @JvmField var heading_kI = 0.0
        @JvmField var heading_kD = 0.2

        @JvmField var translational_kP = 0.5
        @JvmField var translational_kI = 0.0
        @JvmField var translational_kD = 0.02
    }

    private val translationalCoefs = PidCoefs(0.0, 0.0, 0.0)
    private val headingCoefs = PidCoefs(0.0, 0.0, 0.0)

    private fun updateCoefs() {
        headingCoefs.run {
            kP = heading_kP
            kI = heading_kI
            kD = heading_kD
        }

        translationalCoefs.run {
            kP = translational_kP
            kI = translational_kI
            kD = translational_kD
        }
    }

    private val fixpointState: FS = FS {
        G.cmd.launchCommand(Subsystem.DRIVETRAIN.nel()) {
            runPosePidController(
                translationalCoefs = translationalCoefs,
                headingCoefs = headingCoefs,
                input = { G.drive.currentPose.value },
                target = { Pose2d(target_x, target_y, target_heading) },
                output = { setDrivePowers(it.x, it.y, it.heading) }
            )
        }

        loopYieldWhile({ !gamepad1.y }) {
            telemetry["Current State"] = "PID Fixpoint"
        }

        freeControlState
    }

    private val freeControlState: FS = FS {
        G.cmd.launchCommand(Subsystem.DRIVETRAIN.nel()) {
            loopYieldWhile({ true }) {
                setAdjustedDrivePowers(
                    C.driveStrafeX,
                    C.driveStrafeY,
                    C.driveTurn
                )
                telemetry["Current State"] = "Joystick Drive"
            }
        }

        suspendUntil { gamepad1.x }
        fixpointState
    }

    override suspend fun runSuspendOpMode() {
        suspendUntilStart()

        coroutineScope {
            launch { runStateMachine(freeControlState) }

            loopYieldWhile({ true }) {
                updateCoefs()

                val pos = G.drive.currentPose.value

                telemetry["x"] = pos.x
                telemetry["y"] = pos.y
                telemetry["heading"] = pos.heading
            }
        }
    }
}