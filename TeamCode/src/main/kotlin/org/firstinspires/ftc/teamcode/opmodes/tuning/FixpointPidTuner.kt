package org.firstinspires.ftc.teamcode.opmodes.tuning

import arrow.core.merge
import arrow.core.nel
import arrow.fx.coroutines.raceN
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.outoftheboxrobotics.suspendftc.loopYieldWhile
import com.outoftheboxrobotics.suspendftc.suspendUntil
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import kotlinx.coroutines.cancelAndJoin
import kotlinx.coroutines.coroutineScope
import kotlinx.coroutines.launch
import org.firstinspires.ftc.teamcode.RobotState
import org.firstinspires.ftc.teamcode.Subsystem
import org.firstinspires.ftc.teamcode.actions.controllers.PidCoefs
import org.firstinspires.ftc.teamcode.actions.controllers.runPosePidController
import org.firstinspires.ftc.teamcode.actions.hardware.DriveConfig
import org.firstinspires.ftc.teamcode.actions.hardware.currentDrivePose
import org.firstinspires.ftc.teamcode.actions.hardware.launchFixpoint
import org.firstinspires.ftc.teamcode.actions.hardware.setAdjustedDrivePowers
import org.firstinspires.ftc.teamcode.actions.hardware.setDrivePowers
import org.firstinspires.ftc.teamcode.driveLooper
import org.firstinspires.ftc.teamcode.opmodes.RobotOpMode
import org.firstinspires.ftc.teamcode.runStateMachine
import org.firstinspires.ftc.teamcode.util.C
import org.firstinspires.ftc.teamcode.util.FS
import org.firstinspires.ftc.teamcode.util.G
import org.firstinspires.ftc.teamcode.util.launchCommand
import org.firstinspires.ftc.teamcode.util.mainLoop
import org.firstinspires.ftc.teamcode.util.set

@TeleOp
@Config
class FixpointPidTuner : RobotOpMode() {
    companion object {
        @JvmField var target_x = 0.0
        @JvmField var target_y = 0.0
        @JvmField var target_heading = 0.0

        @JvmField var heading_kP = DriveConfig.headingPid.kP
        @JvmField var heading_kI = DriveConfig.headingPid.kI
        @JvmField var heading_kD = DriveConfig.headingPid.kD

        @JvmField var translational_kP = DriveConfig.translationalPid.kP
        @JvmField var translational_kI = DriveConfig.translationalPid.kI
        @JvmField var translational_kD = DriveConfig.translationalPid.kD

        @JvmField var kStaticOffset = 0.05
        @JvmField var targetHz = 30
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

    private val kStaticState: FS = FS {
        G.cmd.launchCommand(Subsystem.DRIVETRAIN.nel()) {
            mainLoop {
                setDrivePowers(0.0, 0.0, kStaticOffset)
            }
        }

        loopYieldWhile({ !gamepad1.y }) {
            telemetry["Current State"] = "kStatic tuning"
        }

        fixpointState
    }

    private val fixpointState: FS = FS {
        G[RobotState.driveLooper].scheduleCoroutine {
            G.cmd.runNewCommand(Subsystem.DRIVETRAIN.nel()) {
                runPosePidController(
                    translationalCoefs = translationalCoefs,
                    headingCoefs = headingCoefs,
                    input = ::currentDrivePose,
                    target = { Pose2d(target_x, target_y, target_heading) },
                    output = { setAdjustedDrivePowers(it.x, it.y, it.heading) },
                    hz = targetHz
                )
            }
        }

        loopYieldWhile({ !gamepad1.b }) {
            telemetry["Current State"] = "PID Fixpoint"
        }

        freeControlState
    }

    private val freeControlState: FS = FS {
        G.cmd.launchCommand(Subsystem.DRIVETRAIN.nel()) {
            mainLoop {
                setAdjustedDrivePowers(
                    C.driveStrafeX,
                    C.driveStrafeY,
                    C.driveTurn
                )
                telemetry["Current State"] = "Joystick Drive"
            }
        }

        raceN(
            coroutineContext,
            {
                suspendUntil { gamepad1.x }
                kStaticState
            },
            {
                suspendUntil { gamepad1.a }
                standardFixpointState
            }
        ).merge()
    }

    private val standardFixpointState: FS = FS {
        val job = launchFixpoint(Pose2d(target_x, target_y, target_heading))

        loopYieldWhile({ !gamepad1.b }) {
            telemetry["Current State"] = "Standard Fixpoint"
        }

        job.cancelAndJoin()
        freeControlState
    }

    override suspend fun runSuspendOpMode() {
        suspendUntilStart()

        coroutineScope {
            launch { runStateMachine(freeControlState) }

            loopYieldWhile({ true }) {
                updateCoefs()

                val pos = currentDrivePose()

                telemetry["x"] = pos.x
                telemetry["y"] = pos.y
                telemetry["heading"] = pos.heading
            }
        }
    }
}