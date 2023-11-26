package org.firstinspires.ftc.teamcode.actions.hardware

import arrow.core.nel
import arrow.fx.coroutines.resourceScope
import arrow.optics.optics
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.util.Angle
import com.outoftheboxrobotics.suspendftc.loopYieldWhile
import com.outoftheboxrobotics.suspendftc.suspendUntil
import com.outoftheboxrobotics.suspendftc.withLooper
import kotlinx.coroutines.ExperimentalCoroutinesApi
import kotlinx.coroutines.async
import kotlinx.coroutines.coroutineScope
import kotlinx.coroutines.launch
import kotlinx.coroutines.supervisorScope
import org.firstinspires.ftc.teamcode.Globals
import org.firstinspires.ftc.teamcode.RobotState
import org.firstinspires.ftc.teamcode.actions.controllers.PidCoefs
import org.firstinspires.ftc.teamcode.actions.controllers.runPosePidController
import org.firstinspires.ftc.teamcode.command.Command
import org.firstinspires.ftc.teamcode.command.Subsystem
import org.firstinspires.ftc.teamcode.driveLooper
import org.firstinspires.ftc.teamcode.driveState
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence
import org.firstinspires.ftc.teamcode.util.C
import org.firstinspires.ftc.teamcode.util.G
import org.firstinspires.ftc.teamcode.util.mainLoop
import kotlin.math.cos
import kotlin.math.sin

@optics
data class DriveState(
    val currentPose: Pose2d,
    val driveControlState: DriveControlState,
    val rrDrive: SampleMecanumDrive? = null
) { companion object }

sealed interface DriveControlState {
    data object Idle : DriveControlState
    data class Fixpoint(val target: Pose2d) : DriveControlState
    data class Following(val trajectory: TrajectorySequence) : DriveControlState
}

object DriveConfig {
    val translationalPid = PidCoefs(0.5, 0.0, 0.02)
    val headingPid = PidCoefs(3.2, 0.0, 0.2)
}

@OptIn(ExperimentalCoroutinesApi::class)
suspend fun runRoadrunnerDrivetrain(rrDrive: SampleMecanumDrive): Nothing = coroutineScope {
    fun imuAngleAsync() = async {
        var n = 0

        loopYieldWhile({ n < 100 }) { n++ }
        nextImuAngle()
    }

    G[RobotState.driveState.rrDrive] = rrDrive

    val poseLens = RobotState.driveState.currentPose

    var nextImuAngle = imuAngleAsync()
    var lastCorrectedPose = G[poseLens]

    var lastSetPose = lastCorrectedPose

    mainLoop {
        val currentState = G[RobotState.driveState]

        if (!(lastSetPose epsilonEquals currentState.currentPose)) {
            val newPose = currentState.currentPose

            rrDrive.poseEstimate = newPose
            lastCorrectedPose = newPose
            nextImuAngle = imuAngleAsync()
        }

        rrDrive.update()
        G[poseLens] = rrDrive.poseEstimate
        lastSetPose = rrDrive.poseEstimate

        nextImuAngle.let {
            if (it.isCompleted) {
                val imuAngle = it.getCompleted()
                val currentPose = G[poseLens]

                val angDelta = Angle.normDelta(imuAngle - currentPose.heading)
                val vecDelta = currentPose.vec() - lastCorrectedPose.vec()

                // Pose estimate will be set in the beginning of the next loop
                G[poseLens] = Pose2d(
                    lastCorrectedPose.vec() + vecDelta.rotated(angDelta),
                    imuAngle
                )
            }
        }
    }
}

fun currentDrivePose() = G[RobotState.driveState.currentPose]

fun launchFixpoint(target: Pose2d) = G[RobotState.driveLooper].scheduleCoroutine {
    Globals.cmd.runNewCommand(Subsystem.DRIVETRAIN.nel()) {
        G[RobotState.driveState.driveControlState] = DriveControlState.Fixpoint(target)

        runPosePidController(
            translationalCoefs = DriveConfig.translationalPid,
            headingCoefs = DriveConfig.headingPid,
            input = { currentDrivePose() },
            target = { target },
            output = { setDrivePowers(it.x, it.y, it.heading) }
        )
    }
}

private suspend fun runDriveCommand(action: suspend () -> Unit) =
    withLooper(G[RobotState.driveLooper]) {
        G.cmd.runNewCommand(Subsystem.DRIVETRAIN.nel(), action)
    }

suspend fun followTrajectory(traj: TrajectorySequence) = runDriveCommand {
    resourceScope {
        G[RobotState.driveState.driveControlState] = DriveControlState.Following(traj)

        val rrDrive = install(
            acquire = {
                requireNotNull(G[RobotState.driveState.rrDrive]) { "rrDrive missing" }
            },
            release = { rrDrive, _ ->
                rrDrive.breakFollowing()
                rrDrive.setMotorPowers(0.0, 0.0, 0.0, 0.0)
            }
        )

        rrDrive.followTrajectorySequenceAsync(traj)

        suspendUntil { !rrDrive.isBusy }
        G[RobotState.driveState.driveControlState] = DriveControlState.Idle
    }
}

suspend fun followTrajectoryFixpoint(traj: TrajectorySequence, stopDist: Double = 0.5) =
    supervisorScope {
        launch { followTrajectory(traj) }
        suspendUntil { currentDrivePose().vec() distTo traj.end().vec() <= stopDist }
        launchFixpoint(traj.end())
    }

private val stopDrivetrainCommand = Command(Subsystem.DRIVETRAIN.nel()) {
    G[RobotState.driveState.driveControlState] = DriveControlState.Idle
}

suspend fun setDrivetrainIdle() = withLooper(G[RobotState.driveLooper]) {
    Globals.cmd.runCommand(stopDrivetrainCommand)
}


fun setDrivePowers(x: Double, y: Double, r: Double) = G.chub.run {
    tr.power = +x +y +r
    tl.power = -x +y +r
    bl.power = -x -y +r
    br.power = +x -y +r
}

fun setAdjustedDrivePowers(x: Double, y: Double, r: Double) {
    val multiplier = 12.0 / G.chub.voltageSensor.voltage

    setDrivePowers(
        multiplier * x,
        multiplier * y * SampleMecanumDrive.LATERAL_MULTIPLIER,
        multiplier * r
    )
}

suspend fun runFieldCentricDrive(): Nothing = mainLoop {
    if (C.imuResetAngle) resetImuAngle()

    val heading = currentImuAngle()

    setDrivePowers(
        C.driveStrafeX * cos(-heading) - C.driveStrafeY * sin(-heading),
        C.driveStrafeX * sin(-heading) + C.driveStrafeY * cos(-heading),
        C.driveTurn
    )
}
