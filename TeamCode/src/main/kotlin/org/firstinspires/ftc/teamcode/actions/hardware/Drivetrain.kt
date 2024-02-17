package org.firstinspires.ftc.teamcode.actions.hardware

import arrow.core.nel
import arrow.fx.coroutines.resourceScope
import arrow.optics.optics
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.path.Path
import com.acmerobotics.roadrunner.util.Angle
import com.outoftheboxrobotics.suspendftc.loopYieldWhile
import com.outoftheboxrobotics.suspendftc.suspendUntil
import com.outoftheboxrobotics.suspendftc.withLooper
import com.outoftheboxrobotics.suspendftc.yieldLooper
import com.qualcomm.robotcore.util.ElapsedTime
import kotlinx.coroutines.ExperimentalCoroutinesApi
import kotlinx.coroutines.async
import kotlinx.coroutines.coroutineScope
import kotlinx.coroutines.supervisorScope
import org.firstinspires.ftc.teamcode.Command
import org.firstinspires.ftc.teamcode.Globals
import org.firstinspires.ftc.teamcode.RobotState
import org.firstinspires.ftc.teamcode.Subsystem
import org.firstinspires.ftc.teamcode.actions.controllers.PidCoefs
import org.firstinspires.ftc.teamcode.actions.controllers.runPosePidController
import org.firstinspires.ftc.teamcode.driveLooper
import org.firstinspires.ftc.teamcode.driveState
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.sequencesegment.TrajectorySegment
import org.firstinspires.ftc.teamcode.util.C
import org.firstinspires.ftc.teamcode.util.G
import org.firstinspires.ftc.teamcode.util.cross
import org.firstinspires.ftc.teamcode.util.mainLoop
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.atan
import kotlin.math.cos
import kotlin.math.sign
import kotlin.math.sin

/**
 * @param newPose New pose that the robot should be set to.
 * @param currentPose Current pose of the robot from the odometry system.
 * @param driveControlState Current state of the drivetrain.
 * @param rrDrive Roadrunner [SampleMecanumDrive] instance.
 */
@optics
data class DriveState(
    val newPose: Pose2d? = null,
    val currentPose: Pose2d,
    val driveControlState: DriveControlState,
    val rrDrive: SampleMecanumDrive? = null
) { companion object }

sealed interface DriveControlState {
    /**
     * No active drivetrain control. Can set motor powers manually.
     */
    data object Idle : DriveControlState

    /**
     * Active pose lock to [target].
     */
    data class Fixpoint(val target: Pose2d) : DriveControlState

    /**
     * Following a [trajectory].
     */
    data class Trajectory(val trajectory: TrajectorySequence) : DriveControlState {
        override fun toString() =
            "Following(trajectory=${trajectory.start()} -> ${trajectory.end()})"
    }

    /**
     * Running an optimized line path follower.
     */
    data class LinePath(
        val start: Vector2d, val end: Vector2d,
        val heading: Double
    ) : DriveControlState
}

object DriveConfig {
    val translationalPid = PidCoefs(0.22, 0.0, 0.037)
    val headingPid = PidCoefs(2.5, 0.0, 0.2)
    const val lookaheadDist = 6.0
    const val kStatic = 0.05
}

/**
 * Main coroutine for running odometry.
 */
@OptIn(ExperimentalCoroutinesApi::class)
suspend fun runRoadrunnerDrivetrain(rrDrive: SampleMecanumDrive): Nothing = coroutineScope {
    val poseLens = RobotState.driveState.currentPose

    // Gets the next IMU angle when the robot is near-stationary.
    fun imuAngleAsync() = async {
        // This is to prevent constantly calling setPoseEstimate()
        repeat(100) { yieldLooper() }

        var lastPose = G[poseLens]
        val timer = ElapsedTime()

        yieldLooper()

        // Wait until the robot is near-stationary
        suspendUntil {
            val dt = timer.seconds()
            timer.reset()

            val poseVel = (-lastPose + G[poseLens].also { lastPose = it }) / dt

            poseVel.vec().norm() <= 5.0 && abs(poseVel.heading) <= PI / 2
        }

        // Poll for next IMU angle
        nextImuAngle()
    }

    G[RobotState.driveState.rrDrive] = rrDrive

    var nextImuAngle = imuAngleAsync()
    var lastCorrectedPose = G[poseLens]

    mainLoop {
        val currentState = G[RobotState.driveState]

        // Check if another coroutine changed the pose, and if so call setPoseEstimate()
        if (currentState.newPose != null) {
            val newPose = currentState.newPose

            rrDrive.poseEstimate = newPose
            lastCorrectedPose = newPose
            nextImuAngle = imuAngleAsync()

            G[RobotState.driveState.nullableNewPose] = null
        }

        rrDrive.update()
        G[poseLens] = rrDrive.poseEstimate

        if (nextImuAngle.isCompleted) {
            // Run the IMU angle correction
            // Essentially, the vector delta from the last corrected pose is rotated to be
            // consistent with the IMU reading

            val imuAngle = nextImuAngle.getCompleted()
            val currentPose = G[poseLens]

            val angDelta = Angle.normDelta(imuAngle - currentPose.heading)
            val vecDelta = currentPose.vec() - lastCorrectedPose.vec()

            // Pose estimate will be set in the beginning of the next loop
            G[RobotState.driveState.newPose] = Pose2d(
                lastCorrectedPose.vec() + vecDelta.rotated(angDelta),
                imuAngle
            )
        }
    }
}

fun currentDrivePose() = G[RobotState.driveState.currentPose]

/**
 * Launches a pose lock to [target].
 */
fun launchFixpoint(target: Pose2d, multiplier: Double = 1.0) = G[RobotState.driveLooper].scheduleCoroutine {
    Globals.cmd.runNewCommand(Subsystem.DRIVETRAIN.nel()) {
        G[RobotState.driveState.driveControlState] = DriveControlState.Fixpoint(target)

        runPosePidController(
            translationalCoefs = DriveConfig.translationalPid,
            headingCoefs = DriveConfig.headingPid,
            input = { currentDrivePose() },
            target = { target },
            output = { setAdjustedDrivePowers(multiplier * it.x, multiplier * it.y, it.heading) }
        )
    }
}

private suspend fun runDriveCommand(action: suspend () -> Unit) =
    withLooper(G[RobotState.driveLooper]) {
        G.cmd.runNewCommand(Subsystem.DRIVETRAIN.nel(), action)
    }

/**
 * Follows a roadrunner [trajectory][traj].
 */
suspend fun followTrajectory(traj: TrajectorySequence) = runDriveCommand {
    resourceScope {
        G[RobotState.driveState.driveControlState] = DriveControlState.Trajectory(traj)

        // Arrow-kt resource dsl
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

suspend fun followTrajectoryPath(
    trajSeq: TrajectorySequence,
    maxPower: () -> Double = { 1.0 },
) {
    val path = List(trajSeq.size()) { trajSeq[it] }
        .filterIsInstance<TrajectorySegment>()
        .flatMap { it.trajectory.path.segments }
        .let { Path(it) }

    var dist = path.project(currentDrivePose().vec()) + DriveConfig.lookaheadDist

    runDriveCommand {
        loopYieldWhile({
            dist = path.fastProject(
                currentDrivePose().vec(),
                dist - DriveConfig.lookaheadDist
            ) + DriveConfig.lookaheadDist

            dist < path.length()
        }) {
            val multiplier = maxPower().coerceIn(0.0..1.0)
            val projected = path[dist]
            val currentPose = currentDrivePose()

            val driveVec = (projected.vec() - currentPose.vec()).rotated(-currentPose.heading)
                .let { it / it.norm() }

            setDrivePowers(
                Pose2d(
                    driveVec * multiplier,
                    DriveConfig.headingPid.kP * Angle.normDelta(
                        projected.heading - currentPose.heading
                    )
                )
            )
        }
    }

    launchFixpoint(path.end())
}

/**
 * Optimized line path follower.
 */
suspend fun followLinePath(
    start: Vector2d,
    end: Vector2d,
    heading: Double,
    maxPower: () -> Double = { 1.0 },
    stopDist: Double = 18.0
) = runDriveCommand {
    G[RobotState.driveState.driveControlState] = DriveControlState.LinePath(start, end, heading)

    // We rotate our drive vector such that it faces a point lookaheadDist ahead of the robot's
    // current position projected onto the line.
    suspendUntil {
        val pose = currentDrivePose()
        val multiplier = maxPower().coerceIn(0.0..1.0)

        val lineVec = end - start
        val startToPose = pose.vec() - start

        val dist = startToPose cross lineVec / lineVec.norm()

        val driveHeading = lineVec.angle() + atan(dist / DriveConfig.lookaheadDist)
        val headingError = Angle.normDelta(heading - pose.heading)

        // TODO Account for non-zero heading (?)
        val poseVel = Pose2d(
            Vector2d.polar(multiplier, driveHeading + headingError - heading),
            headingError * DriveConfig.headingPid.kP
        )

        setDrivePowers(poseVel)

        (end - pose.vec()) dot lineVec / lineVec.norm() <= stopDist
    }

    setDrivePowers(0.0, 0.0, 0.0)
    G[RobotState.driveState.driveControlState] = DriveControlState.Idle
}

/**
 * Follows a [trajectory][traj] and then launches a pose lock to the end of the trajectory.
 */
suspend fun followTrajectoryFixpoint(
    traj: TrajectorySequence,
    stopDist: Double = 0.5
) =
    supervisorScope {
//        launch { followTrajectory(traj) }
//        suspendUntil { currentDrivePose().vec() distTo traj.end().vec() <= stopDist }

        followTrajectory(traj)
        launchFixpoint(traj.end())
    }

/**
 * Follows a line path from the robot's current position to [target].
 */
suspend fun lineTo(target: Pose2d, maxPower: () -> Double = { 1.0 }, stopDist: Double = 18.0) =
    followLinePath(currentDrivePose().vec(), target.vec(), target.heading, maxPower, stopDist)

private val stopDrivetrainCommand = Command(Subsystem.DRIVETRAIN.nel()) {
    G[RobotState.driveState.driveControlState] = DriveControlState.Idle
}

suspend fun setDrivetrainIdle() = withLooper(G[RobotState.driveLooper]) {
    Globals.cmd.runCommand(stopDrivetrainCommand)
}

/**
 * Manually set the drivetrain robot-relative fwd, strafe, and turn powers.
 */
fun setDrivePowers(x: Double, y: Double, r: Double, useKStatic: Boolean = false) = G.chub.run {
    fun Double.applyKStatic() =
        if (useKStatic) this + DriveConfig.kStatic * sign(this)
        else this

    tr.power = (+x +y +r).applyKStatic()
    tl.power = (-x +y +r).applyKStatic()
    bl.power = (-x -y +r).applyKStatic()
    br.power = (+x -y +r).applyKStatic()
}

/**
 * Manually set the drivetrain robot-relative fwd, strafe, and turn powers.
 */
fun setDrivePowers(poseVel: Pose2d) = setDrivePowers(poseVel.x, poseVel.y, poseVel.heading)

/**
 * Manually set the drivetrain field-relative fwd, strafe, and turn powers with
 * voltage compensation, strafe multiplier, and kStatic.
 */
fun setAdjustedDrivePowers(x: Double, y: Double, r: Double) {
    val multiplier = 12.0 / G.chub.voltageSensor.voltage

    setDrivePowers(
        multiplier * x,
        multiplier * y * SampleMecanumDrive.LATERAL_MULTIPLIER,
        multiplier * r,
        true
    )
}

/**
 * Runs a field-centric drive loop.
 */
suspend fun runFieldCentricDrive(): Nothing = mainLoop {
    if (C.imuResetAngle) resetImuAngle()

    val heading = currentImuAngle()

    val multiplier = if (C.slowDrive) 0.3 else 1.0

    setDrivePowers(
        multiplier * (C.driveStrafeX * cos(-heading) - C.driveStrafeY * sin(-heading)),
        SampleMecanumDrive.LATERAL_MULTIPLIER * multiplier * (C.driveStrafeX * sin(-heading) + C.driveStrafeY * cos(-heading)),
        multiplier * C.driveTurn
    )
}
