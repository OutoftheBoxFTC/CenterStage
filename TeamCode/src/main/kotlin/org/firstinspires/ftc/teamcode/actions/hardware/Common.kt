package org.firstinspires.ftc.teamcode.actions.hardware

import arrow.core.merge
import arrow.core.nel
import arrow.core.toNonEmptyListOrNull
import arrow.fx.coroutines.raceN
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.util.Angle
import com.outoftheboxrobotics.suspendftc.suspendFor
import com.outoftheboxrobotics.suspendftc.suspendUntil
import com.outoftheboxrobotics.suspendftc.yieldLooper
import com.qualcomm.robotcore.util.ElapsedTime
import com.qualcomm.robotcore.util.RobotLog
import kotlinx.coroutines.Job
import kotlinx.coroutines.cancelAndJoin
import kotlinx.coroutines.coroutineScope
import kotlinx.coroutines.flow.first
import kotlinx.coroutines.flow.mapNotNull
import kotlinx.coroutines.flow.withIndex
import kotlinx.coroutines.launch
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference
import org.firstinspires.ftc.robotcore.external.navigation.Orientation
import org.firstinspires.ftc.teamcode.Globals
import org.firstinspires.ftc.teamcode.RobotState
import org.firstinspires.ftc.teamcode.Subsystem
import org.firstinspires.ftc.teamcode.actions.controllers.PidCoefs
import org.firstinspires.ftc.teamcode.actions.controllers.runPidController
import org.firstinspires.ftc.teamcode.actions.controllers.runPosePidController
import org.firstinspires.ftc.teamcode.actions.controllers.runVeloPid
import org.firstinspires.ftc.teamcode.driveLooper
import org.firstinspires.ftc.teamcode.driveState
import org.firstinspires.ftc.teamcode.opmodes.tuning.ExtensionVeloPidTuner
import org.firstinspires.ftc.teamcode.util.G
import org.firstinspires.ftc.teamcode.util.cross
import org.firstinspires.ftc.teamcode.util.mainLoop
import org.firstinspires.ftc.teamcode.util.mapState
import org.firstinspires.ftc.teamcode.vision.PreloadDetectionPipeline
import kotlin.coroutines.coroutineContext
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.exp
import kotlin.math.sign

/**
 * Resets the robot's pose to the given pose, or the origin if no pose is given.
 */
fun resetDrivePose(newPose: Pose2d = Pose2d()) {
    resetImuAngle(newPose.heading)
    G[RobotState.driveState.newPose] = newPose
}

/**
 * Runs the intake transfer sequence.
 */
suspend fun intakeTransfer(
    finalArmPos: Double = ArmPosition.OUTTAKE.pos,
    finalLiftPos: Int? = null
): Unit = coroutineScope {
    retractLift()

    openClaws()
    suspendFor(100)

    G.ehub.intakeRoller.power = -0.8

    coroutineScope {
        launch {
            retractExtension()
            profileArm(ArmPosition.TRANSFER)
        }
        suspendFor(100)
        setTiltPosition(IntakeTiltPosition.TRANSFER_FLAT)
    }

    setTiltPosition(IntakeTiltPosition.TRANSFER.pos + 0.15)

    suspendFor(500)

    setTiltPosition(IntakeTiltPosition.TRANSFER)
    G.ehub.intakeRoller.power = 0.0

    setClawPos(ClawPosition.BLACK_CLOSE)

    G.ehub.outtakeLift.power = -1.0
    G.ehub.extension.power = -1.0

    suspendFor(300)

    closeClaws()

    suspendFor(450)

    G.ehub.outtakeLift.power = 0.0
    G.ehub.extension.power = -0.5

    setArmPosition(ArmPosition.TRANSFER.pos + 0.01)

    setTiltPosition(IntakeTiltPosition.POST_TRANSFER)
    G.ehub.intakeRoller.power = -0.8
    suspendFor(50)
    G.ehub.intakeRoller.power = 0.0

    suspendFor(100)

    liftUpTo(LiftConfig.transferHeightMin)
    retractExtension()

    setTiltPosition(IntakeTiltPosition.HIGH)

    val armJob = launch {
        profileArm(finalArmPos)
    }

    launch {
        if (finalLiftPos != null) liftUpTo(finalLiftPos)
        else armJob.join()
    }
}

suspend fun nextBackboardApriltagPosition(): Pose2d {
    var aprilTagDist = 6.0

    fun recalcAprilTagDist(tagPoses: List<Pair<Int, Pose2d>>): Double? {
        val distances = mutableListOf<Double>()

        repeat(tagPoses.size) { i ->
            for (j in i..<tagPoses.size) {
                if (i == j) continue
                if ((i < 4) xor (j < 4)) continue

                val (i1, p1) = tagPoses[i]
                val (i2, p2) = tagPoses[j]

                distances.add((p2 - p1).vec().norm() / abs(i2 - i1))
            }
        }

        return distances.toNonEmptyListOrNull()?.average()
    }

    val poses = G.robotState
        .mapState { it.visionState.aprilTagDetections }
        // Wait for next non-empty detection list
        .mapNotNull { detections ->
            detections.filter { it.id in 1..6 }.toNonEmptyListOrNull()
        }
        .withIndex()
        .first { it.index > 0 }
        .value
        // Get apriltag Pose2d
        .map {
            val pose = Pose2d(
                it.pose.z,
                -it.pose.x,
                -Orientation.getOrientation(
                    it.pose.R,
                    AxesReference.INTRINSIC,
                    AxesOrder.YXZ,
                    AngleUnit.RADIANS
                ).firstAngle.toDouble()
            )

            it.id to pose
        }
        // Reset apriltag distance if multiple found
        .also {
            aprilTagDist = recalcAprilTagDist(it) ?: return@also
        }
        .map { (id, pose) ->
            pose + Pose2d(
                pose.headingVec().rotated(PI / 2) * when (id) {
                    6, 3 -> aprilTagDist
                    1, 4 -> -aprilTagDist
                    else -> 0.0
                },
                0.0
            )
        }

    // Average from all apriltags
    val backboard = Pose2d(
        poses.map { it.x }.average(),
        poses.map { it.y }.average(),
        poses.map { it.heading }.average()
    ).plus(
        Pose2d(
            7.91,
            3.0,
            0.0
        )
    ).let {
        Pose2d(it.vec().rotated(PI), it.heading)
    }

    val currentPose = currentDrivePose()

    return Pose2d(
        currentPose.vec() + backboard.vec().rotated(currentPose.heading),
        backboard.heading + currentPose.heading
    )
}

suspend fun launchOuttakeFixpoint(
    estimate: Pose2d,
    target: PreloadDetectionPipeline.RandomizationPosition?,
    expectedBackstagePos: Pose2d? = null,
    updateBackstagePos: (Pose2d) -> Unit = {},
): Pair<Job, Pose2d> = coroutineScope {
    val targetPose = raceN(
        coroutineContext,
        {
            val detectedBackstagePos = nextBackboardApriltagPosition()

            if (expectedBackstagePos != null) {
                val vecDelta = expectedBackstagePos.vec() - detectedBackstagePos.vec()

                RobotLog.i("Apriltag relocalization (delta=$vecDelta)")

                G[RobotState.driveState.nullableNewPose] = Pose2d(
                    currentDrivePose().vec() + vecDelta,
                    currentDrivePose().heading
                )
            } else {
                updateBackstagePos(detectedBackstagePos)
            }

            val backstagePos = expectedBackstagePos ?: detectedBackstagePos

            val hvec = backstagePos.headingVec()

            Pose2d(
                backstagePos.vec() + hvec * 16.8 + hvec.rotated(PI / 2) * when (target) {
                    PreloadDetectionPipeline.RandomizationPosition.LEFT -> -6.0
                    PreloadDetectionPipeline.RandomizationPosition.CENTER -> 0.0
                    PreloadDetectionPipeline.RandomizationPosition.RIGHT -> 6.0
                    null -> 0.0
                },
                backstagePos.heading
            )
        },
        {
            suspendFor(500)
            RobotLog.w("No apriltag detections for outtake, using estimate.")
            estimate
        }
    ).merge()

    val job = G[RobotState.driveLooper].scheduleCoroutine {
        Globals.cmd.runNewCommand(Subsystem.DRIVETRAIN.nel()) {
            G[RobotState.driveState.driveControlState] = DriveControlState.Fixpoint(targetPose)

            runPosePidController(
                translationalCoefs = DriveConfig.translationalPid,
                headingCoefs = DriveConfig.headingPid,
                input = { currentDrivePose() },
                target = { targetPose },
                output = { setAdjustedDrivePowers(
                    if (targetPose.headingVec() dot (currentDrivePose() - targetPose).vec() < 0.0) 0.0 else 0.5 * it.x,
                    if (target != null) it.y else 0.0,
                    it.heading
                ) }
            )
        }
    }

    job to targetPose
}

suspend fun scoreOnBackstage(
    estimate: Pose2d,
    target: PreloadDetectionPipeline.RandomizationPosition?,
    preOuttake: Pose2d = currentDrivePose(),
    expectedBackstagePos: Pose2d? = null,
    updateBackstagePos: (Pose2d) -> Unit = {},
    precisePlace: Boolean = false
): Job {
    val (job, robotTarget) = launchOuttakeFixpoint(estimate, target, expectedBackstagePos, updateBackstagePos)

    val returnPos = Pose2d(
        robotTarget.vec() + robotTarget.headingVec() * preOuttake.vec().distTo(robotTarget.vec()),
        preOuttake.heading
    )

    raceN(
        coroutineContext,
        {
            suspendFor(2500)
        },
        {

            if (target != null)
                suspendUntil { currentDrivePose().vec().distTo(robotTarget.vec()) < 1.0 }
            else
                suspendUntil { abs((robotTarget.vec() - currentDrivePose().vec()) dot currentDrivePose().headingVec()) < 1.0 }
            suspendFor(200)
        }
    )

    if (precisePlace) {
        liftDown()
        suspendFor(50)
        liftHold()
    }

    openClaws()
    job.cancelAndJoin()

    suspendFor(300)

    return launchFixpoint(returnPos, multiplier = 0.8)
}

suspend inline fun intakeFixpoint(
    crossinline intakeMultiplier: () -> Double = { 1.0 },
    crossinline target: () -> Vector2d,
    crossinline headingOutput: (Double) -> Unit
): Nothing = coroutineScope {
    var perpError = 0.0
    var turnPower = 0.0
    var extensionTarget = 0.0

    launch {
        mainLoop {
            val extendoVec = G[RobotState.driveState.extendoPose].vec()
            val driveVec = currentDrivePose().vec()

            val dir = (target() - driveVec).let {
                extensionTarget = (it.norm() - DriveConfig.intakeOdoRadius) / DriveConfig.intakeOdoExtensionMultiplier
                it / it.norm()
            }

            perpError = (target() - extendoVec) cross dir
        }
    }

    launch {
        runPidController(
            coefs = PidCoefs(0.5, 0.0, 0.0),
            input = { 0.0 },
            target = { perpError },
            output = { G.ehub.intakeWheel.power = -it },  // Don't worry about it
            hz = 30
        )
    }

    launch {
        runVeloPid(
            feedforward = ExtensionConfig.extensionFeedforward,
            pid = ExtensionConfig.extensionVeloPid,
            input = { extensionLength().toDouble() },
            target = { extensionTarget },
            output = { G.ehub.extension.power = it * intakeMultiplier.invoke() },
            maxVel = ExtensionVeloPidTuner.maxVel,
            maxAccel = ExtensionVeloPidTuner.maxAccel,
            hz = 30
        )
    }

    launch {
        runPidController(
            coefs = DriveConfig.headingPid,
            input = { 0.0 },
            target = {
                -perpError / (extensionLength() * DriveConfig.intakeOdoExtensionMultiplier + DriveConfig.intakeOdoRadius) +
                0.08 * G.ehub.extension.power.coerceAtLeast(0.0) * extensionLength() * DriveConfig.intakeOdoExtensionMultiplier
            },
            output = { turnPower = it },
            hz = 30
        )
    }

    mainLoop {
        headingOutput.invoke(turnPower)
    }
}

suspend fun dualFixpoint(
    extensionMultiplier: Double = 1.0,
    robotMultiplier: Double = 1.0,
    intakeTarget: Vector2d,
    robotTarget: Vector2d,
    intakeHeadingConstraintMultiplier: Double = 0.0
): Nothing = coroutineScope {
    val targetHeading = (intakeTarget - robotTarget).angle()

    val fixpointJob = launchSmoothStop(
        target = Pose2d(robotTarget, targetHeading),
        multiplier = robotMultiplier
    )

    try {
        intakeFixpoint(
            intakeMultiplier = {
                val headingError = Angle.normDelta(currentDrivePose().heading - targetHeading)
                extensionMultiplier * exp(-intakeHeadingConstraintMultiplier * abs(headingError))
            },
            target = { intakeTarget },
            headingOutput = {}
        )
    } finally {
        fixpointJob.cancel()
    }
}