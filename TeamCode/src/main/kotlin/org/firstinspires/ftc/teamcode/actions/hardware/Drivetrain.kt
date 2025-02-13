package org.firstinspires.ftc.teamcode.actions.hardware

import android.util.Log
import arrow.core.nel
import arrow.fx.coroutines.raceN
import arrow.fx.coroutines.resourceScope
import arrow.fx.coroutines.timeInMillis
import arrow.optics.optics
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.path.Path
import com.acmerobotics.roadrunner.util.Angle
import com.acmerobotics.roadrunner.util.epsilonEquals
import com.outoftheboxrobotics.suspendftc.loopYieldWhile
import com.outoftheboxrobotics.suspendftc.receiveAndYield
import com.outoftheboxrobotics.suspendftc.suspendFor
import com.outoftheboxrobotics.suspendftc.suspendUntil
import com.outoftheboxrobotics.suspendftc.withLooper
import com.outoftheboxrobotics.suspendftc.yieldLooper
import com.qualcomm.robotcore.util.ElapsedTime
import com.qualcomm.robotcore.util.RobotLog
import kotlinx.coroutines.ExperimentalCoroutinesApi
import kotlinx.coroutines.async
import kotlinx.coroutines.cancelAndJoin
import kotlinx.coroutines.channels.Channel
import kotlinx.coroutines.coroutineScope
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.update
import kotlinx.coroutines.launch
import kotlinx.coroutines.supervisorScope
import org.firstinspires.ftc.teamcode.Command
import org.firstinspires.ftc.teamcode.Globals
import org.firstinspires.ftc.teamcode.RobotState
import org.firstinspires.ftc.teamcode.Subsystem
import org.firstinspires.ftc.teamcode.actions.controllers.PidCoefs
import org.firstinspires.ftc.teamcode.actions.controllers.runPosePidController
import org.firstinspires.ftc.teamcode.actions.controllers.runSmoothStopPid
import org.firstinspires.ftc.teamcode.driveLooper
import org.firstinspires.ftc.teamcode.driveState
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.sequencesegment.TrajectorySegment
import org.firstinspires.ftc.teamcode.util.C
import org.firstinspires.ftc.teamcode.util.G
import org.firstinspires.ftc.teamcode.util.cross
import org.firstinspires.ftc.teamcode.util.ilerp
import org.firstinspires.ftc.teamcode.util.lerp
import org.firstinspires.ftc.teamcode.util.mainLoop
import org.firstinspires.ftc.teamcode.util.mapState
import org.firstinspires.ftc.teamcode.util.modify
import org.firstinspires.ftc.teamcode.util.next
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.atan
import kotlin.math.cos
import kotlin.math.pow
import kotlin.math.sign
import kotlin.math.sin
import kotlin.math.sqrt

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
    val extendoPose: Pose2d,
    val driveControlState: DriveControlState,
    val imuCorrectionRequests: Channel<Unit> = Channel(Channel.RENDEZVOUS),
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
    const val oldFollowerLookaheadDist = 6.0
    const val minFollowerDist = 6.0
    const val curvatureLookaheadDist = 15.0
    const val kStatic = 0.05

    const val intakeOdoMultiplier = 63.837375232725385 / 102867
    const val intakeOdoRadius = 5.7064
    const val intakeOdoExtensionMultiplier = 48.0/953

    const val kN = 1.0 / 6.0
    const val kC = 0.2

    val smoothStopPid = PidCoefs(0.2, 0.0, 0.008)
    const val smoothStopAccel = 86.8601797527
}

/**
 * Main coroutine for running odometry.
 */
@OptIn(ExperimentalCoroutinesApi::class)
suspend fun runRoadrunnerDrivetrain(rrDrive: SampleMecanumDrive): Nothing = coroutineScope {
    val poseLens = RobotState.driveState.currentPose

    // Current, Corrected
    val imuCorrectionState = MutableStateFlow(G[poseLens] to G[poseLens])

    launch {
        val drivePoseUpdates = MutableStateFlow(G[poseLens] to System.nanoTime())

        launch {
            G.robotState.mapState { it.driveState.currentPose }.collect {
                drivePoseUpdates.value = it to System.nanoTime()
            }
        }

        launch {
            while (true) {
                raceN(
                    coroutineContext,
                    {
                        G[RobotState.driveState.imuCorrectionRequests].receiveAndYield()
                    },
                    {
                        repeat(10) { yieldLooper() }
                    }
                )

                val (current, newPose) = imuCorrectionState.next()

                val extendoPoseDelta =
                    Angle.normDelta(newPose.heading - current.heading) *
                    (DriveConfig.intakeOdoRadius + extensionLength() * DriveConfig.intakeOdoExtensionMultiplier)

                if (!G.chub.extensionLimitSwitch && abs(extendoPoseDelta) > 3.0) {
                    RobotLog.i("Skipping IMU Odo Correction (extendo delta=$extendoPoseDelta")

                    continue
                }

                if (G[RobotState.driveState.nullableNewPose] != null) continue

                RobotLog.i("IMU Odo Correction (delta=${Angle.normDelta(newPose.heading - current.heading) * 180 / PI} deg")

                G[RobotState.driveState.nullableNewPose] = newPose
            }
        }

        while (true) {
            // Run the IMU angle correction
            // Essentially, the vector delta from the last corrected pose is rotated to be
            // consistent with the IMU reading

            val lastState = imuCorrectionState.value

            val imuAngle = nextImuAngle()
            val imuTime = System.nanoTime()

            val (d0, t0) = drivePoseUpdates.value
            val (d1, t1) = drivePoseUpdates.next()

            // RobotLog.i("IMU read, t0=$t0, imu=$imuTime, t1=$t1")

            val (lastPose, lastCorrection) = imuCorrectionState.value

            val expectedPose = Pose2d(
                lastPose.vec() + (d1.vec() - lastPose.vec()).rotated(lastCorrection.heading - lastPose.heading),
                d1.heading + (lastCorrection.heading - lastPose.heading)
            )

            val angDelta = imuAngle - lerp(
                d0.heading, d1.heading, ilerp(t0.toDouble(), t1.toDouble(), imuTime.toDouble())
            ) - (lastCorrection.heading - lastPose.heading)

            val vecDelta = expectedPose.vec() - lastCorrection.vec()

            if (lastState != imuCorrectionState.value) continue

            imuCorrectionState.value = d1 to Pose2d(
                lastCorrection.vec() + vecDelta.rotated(angDelta),
                imuAngle
            )
        }
    }

    G[RobotState.driveState.rrDrive] = rrDrive

    // Extendo curvature
    var k = 0.0

    var lastPose = G[poseLens]
    var lastOdoPos = G.chub.odoIntake.currentPosition

    var lastRelPoseDelta = Pose2d()

    yieldLooper()

    mainLoop {
        val currentState = G[RobotState.driveState]

        // Check if another coroutine changed the pose, and if so call setPoseEstimate()
        if (currentState.newPose != null) {
            val newPose = currentState.newPose + Pose2d(
                lastRelPoseDelta.vec().rotated(currentState.newPose.heading),
                lastRelPoseDelta.heading
            )

            rrDrive.poseEstimate = newPose
            imuCorrectionState.value = newPose to newPose

            G[RobotState.driveState.nullableNewPose] = null
        }

        rrDrive.update()

        lastRelPoseDelta = (rrDrive.poseEstimate - G[poseLens]).let {
            Pose2d(it.vec().rotated(-G[poseLens].heading), it.heading)
        }

        G[poseLens] = rrDrive.poseEstimate

        // Constant Curvature Extendometry
        // Good luck.
        val drivePose = G[poseLens]
        val robotPoseDelta = drivePose - lastPose
        val odoPosDelta = (G.chub.odoIntake.currentPosition - lastOdoPos) * DriveConfig.intakeOdoMultiplier

        lastOdoPos = G.chub.odoIntake.currentPosition
        lastPose = drivePose

        if (G.chub.extensionLimitSwitch) {
            G[RobotState.driveState.extendoPose] = Pose2d(
                drivePose.vec() + Vector2d.polar(DriveConfig.intakeOdoRadius, drivePose.heading),
                drivePose.heading
            )

            k = 0.0
        } else {
            val h = extensionLength() * DriveConfig.intakeOdoExtensionMultiplier + DriveConfig.intakeOdoRadius
            val mHat = drivePose.headingVec().rotated(h*k + PI / 2)

            val r = robotPoseDelta.vec() dot mHat
            val rot =
                if (k epsilonEquals 0.0)
                    h * Angle.normDelta(robotPoseDelta.heading)
                else
                    cos(h * k / 2) * sqrt(2 * (1 - cos(h*k)) / k.pow(2)) *
                    Angle.normDelta(robotPoseDelta.heading)

            val dqdk = (
                if (k epsilonEquals 0.0)
                    Vector2d(0.0, h*h/2)
                else
                    Vector2d(
                        (h*k*cos(h*k) - sin(h*k)) / (k*k),
                        (h*k*sin(h*k) + cos(h*k) - 1) / (k*k)
                    )
            ).rotated(drivePose.heading) dot mHat

            if (!(dqdk epsilonEquals 0.0))
                k += (odoPosDelta - (r + rot)) / dqdk

            if (extensionLength() < 1000)
                k = 0.0

            G[RobotState.driveState.extendoPose] = drivePose +
                if (k epsilonEquals 0.0) Pose2d(Vector2d(h, 0.0).rotated(drivePose.heading), 0.0)
                else Pose2d(
                    Vector2d(sin(h*k)/k, (1-cos(h*k))/k).rotated(drivePose.heading),
                    h*k
                )
        }

        if (G.chub.extensionLimitSwitch) resetExtensionLength()
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

fun launchSmoothStop(target: Pose2d, multiplier: Double = 1.0) = G[RobotState.driveLooper].scheduleCoroutine {
    G.cmd.runNewCommand(Subsystem.DRIVETRAIN.nel()) {
        G[RobotState.driveState.driveControlState] = DriveControlState.Fixpoint(target)

        runSmoothStopPid(
            smoothStopCoefs = DriveConfig.smoothStopPid,
            headingCoefs = DriveConfig.headingPid,
            smoothStopAccel = DriveConfig.smoothStopAccel,
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

suspend fun followTrajectoryPathOld(
    trajSeq: TrajectorySequence,
    maxPower: () -> Double = { 1.0 },
) {
    val path = List(trajSeq.size()) { trajSeq[it] }
        .filterIsInstance<TrajectorySegment>()
        .flatMap { it.trajectory.path.segments }
        .let { Path(it) }

    var dist = path.project(currentDrivePose().vec()) + DriveConfig.oldFollowerLookaheadDist

    runDriveCommand {
        loopYieldWhile({
            dist = path.fastProject(
                currentDrivePose().vec(),
                dist - DriveConfig.oldFollowerLookaheadDist
            ) + DriveConfig.oldFollowerLookaheadDist

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

suspend fun followTrajectoryPath(
    trajSeq: TrajectorySequence,
    maxPower: () -> Double = { 1.0 },
    headingPower: (()-> Double)? = null
) {
    val path = List(trajSeq.size()) { trajSeq[it] }
        .filterIsInstance<TrajectorySegment>()
        .flatMap { it.trajectory.path.segments }
        .let { Path(it) }

    var dist = path.project(currentDrivePose().vec())

    runDriveCommand {
        loopYieldWhile({
            dist = path.fastProject(currentDrivePose().vec(), dist)

            dist < path.length() - DriveConfig.minFollowerDist
        }) {
            val multiplier = maxPower().coerceIn(0.0..1.0)
            val projected = path[dist]

            val tangent = path.deriv(dist).vec()

            val currentPose = currentDrivePose()

            val v = G[RobotState.driveState.rrDrive]!!.poseVelocity!!.vec().norm()

            val norm = (currentPose.vec() - projected.vec()) cross tangent
            val kappa = tangent cross path.secondDeriv(dist + DriveConfig.curvatureLookaheadDist).vec()

            val driveVec = Vector2d.polar(
                multiplier,
                tangent.angle()
                        + atan(DriveConfig.kN * norm + DriveConfig.kC * kappa * v * v)
                        - currentPose.heading
            )

            setDrivePowers(
                Pose2d(
                    driveVec * multiplier,
                    headingPower?.invoke() ?: (DriveConfig.headingPid.kP * Angle.normDelta(
                        projected.heading - currentPose.heading
                    ))
                )
            )
        }
    }

    launchSmoothStop(path.end())
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

        val driveHeading = lineVec.angle() + atan(dist / DriveConfig.oldFollowerLookaheadDist)
        val headingError = Angle.normDelta(heading - pose.heading)

        // TODO Account for non-zero heading (?)
        val poseVel = Pose2d(
            Vector2d.polar(multiplier, driveHeading + headingError - heading),
            headingError * DriveConfig.headingPid.kP
        )

        setDrivePowers(poseVel)

        (end - pose.vec()) dot lineVec / lineVec.norm() <= stopDist
    }

    ezBrake()
    G[RobotState.driveState.driveControlState] = DriveControlState.Idle
}

/**
 * Follows a [trajectory][traj] and then launches a pose lock to the end of the trajectory.
 */
suspend fun followTrajectoryFixpoint(
    traj: TrajectorySequence,
    stopTime: Long = 500
) =
    supervisorScope {
//        launch { followTrajectory(traj) }
//        suspendUntil { currentDrivePose().vec() distTo traj.end().vec() <= stopDist }

        val job = launch { followTrajectory(traj) }

        val timer = ElapsedTime()
        timer.reset()

        suspendUntil { job.isCompleted || timer.milliseconds() + stopTime > traj.duration() * 1000 }
        job.cancelAndJoin()

        launchSmoothStop(traj.end())
    }

/**
 * Follows a line path from the robot's current position to [target].
 */
suspend fun lineTo(target: Pose2d, maxPower: () -> Double = { 1.0 }, stopDist: Double = 18.0) =
    followLinePath(currentDrivePose().vec(), target.vec(), target.heading, maxPower, stopDist)

suspend fun ezBrake() {
    setDrivePowers(
        Pose2d(
            G[RobotState.driveState.rrDrive]!!.poseVelocity!!.vec().let { -it / it.norm() },
            0.0
        )
    )

    suspendFor(50)

    setDrivePowers(0.0, 0.0, 0.0)
}

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
