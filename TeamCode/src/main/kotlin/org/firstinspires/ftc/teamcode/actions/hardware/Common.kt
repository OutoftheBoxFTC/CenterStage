package org.firstinspires.ftc.teamcode.actions.hardware

import arrow.core.merge
import arrow.core.nel
import arrow.core.toNonEmptyListOrNull
import arrow.fx.coroutines.raceN
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.outoftheboxrobotics.suspendftc.suspendFor
import com.outoftheboxrobotics.suspendftc.suspendUntil
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
import org.firstinspires.ftc.teamcode.driveLooper
import org.firstinspires.ftc.teamcode.driveState
import org.firstinspires.ftc.teamcode.util.G
import org.firstinspires.ftc.teamcode.util.cross
import org.firstinspires.ftc.teamcode.util.mainLoop
import org.firstinspires.ftc.teamcode.util.mapState
import org.firstinspires.ftc.teamcode.vision.PreloadDetectionPipeline
import kotlin.coroutines.coroutineContext
import kotlin.math.PI
import kotlin.math.abs

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
) = coroutineScope {
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

    suspendFor(300)

    setTiltPosition(IntakeTiltPosition.TRANSFER)
    G.ehub.intakeRoller.power = 0.0

    setClawPos(ClawPosition.BLACK_CLOSE)

    G.ehub.outtakeLift.power = -1.0
    G.ehub.extension.power = -1.0

    suspendFor(200)

    closeClaws()

    suspendFor(350)

    G.ehub.outtakeLift.power = 0.0
    G.ehub.extension.power = -0.5

    setArmPosition(ArmPosition.TRANSFER.pos + 0.01)

    setTiltPosition(IntakeTiltPosition.POST_TRANSFER)
    G.ehub.intakeRoller.power = -0.8
    suspendFor(50)
    G.ehub.intakeRoller.power = 0.0


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
    target: PreloadDetectionPipeline.RandomizationPosition
): Pair<Job, Pose2d> = coroutineScope {
    val targetPose = raceN(
        coroutineContext,
        {
            val backstagePos = nextBackboardApriltagPosition()
            val hvec = backstagePos.headingVec()

            Pose2d(
                backstagePos.vec() + hvec * 16.8 + hvec.rotated(PI / 2) * when (target) {
                     PreloadDetectionPipeline.RandomizationPosition.LEFT -> -6.0
                     PreloadDetectionPipeline.RandomizationPosition.CENTER -> 0.0
                     PreloadDetectionPipeline.RandomizationPosition.RIGHT -> 6.0
                },
                backstagePos.heading
            )
        },
        {
            suspendFor(500)
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
                    it.y,
                    it.heading
                ) }
            )
        }
    }

    job to targetPose
}

suspend fun scoreOnBackstage(
    estimate: Pose2d,
    target: PreloadDetectionPipeline.RandomizationPosition,
    preOuttake: Pose2d = currentDrivePose()
): Job {
    val (job, target) = launchOuttakeFixpoint(estimate, target)

    val returnPos = Pose2d(
        target.vec() + target.headingVec() * preOuttake.vec().distTo(target.vec()),
        preOuttake.heading
    )

    raceN(
        coroutineContext,
        {
            suspendFor(2000)
        },
        {
            suspendUntil { currentDrivePose().vec().distTo(target.vec()) < 1.0 }
            suspendFor(200)
        }
    )

    openClaws()
    job.cancelAndJoin()

    suspendFor(200)

    return launchFixpoint(returnPos, multiplier = 0.8)
}

suspend fun intakeFixpoint(
    intakePid: PidCoefs = PidCoefs(0.5, 0.0, 0.0),
    target: () -> Vector2d,
    headingOutput: (Double) -> Unit
): Nothing = coroutineScope {
    var parError = 0.0
    var perpError = 0.0

    var turnPower = 0.0

    launch {
        mainLoop {
            val extendoPose = G[RobotState.driveState.extendoPose]
            val drivePose = currentDrivePose()

            parError = (target() - extendoPose.vec()) dot (target() - drivePose.vec()).let { it / it.norm() }
            perpError = (target() - extendoPose.vec()) cross (target() - drivePose.vec()).let { it / it.norm() }
        }
    }

    launch {
        runPidController(
            coefs = intakePid,
            input = { 0.0 },
            target = { perpError },
            output = { G.chub.intakeWheel.power = -it },  // Don't worry about it
            hz = 30
        )
    }

    launch {
        runPidController(
            coefs = ExtensionConfig.pidCoefs,
            input = { 0.0 },
            target = { parError / DriveConfig.intakeOdoExtensionMultiplier },
            output = { setExtensionPower(it) },
            tolerance = 10.0,
            hz = 30
        )
    }

    launch {
        runPidController(
            coefs = DriveConfig.headingPid,
            input = { 0.0 },
            target = { -perpError / (extensionLength() * DriveConfig.intakeOdoExtensionMultiplier + DriveConfig.intakeOdoRadius) },
            output = { turnPower = it },
            hz = 30
        )
    }

    mainLoop {
        headingOutput.invoke(turnPower)
    }
}

suspend fun dualFixpoint(
    intakePid: PidCoefs = PidCoefs(0.5, 0.0, 0.0),
    intakeTarget: Vector2d,
    robotTarget: Vector2d
): Nothing = coroutineScope {
    val fixpointJob = launchFixpoint(
        target = Pose2d(robotTarget, (intakeTarget - robotTarget).angle())
    )

    try {
        intakeFixpoint(
            intakePid = intakePid,
            target = { intakeTarget },
            headingOutput = {}
        )
    } finally {
        fixpointJob.cancel()
    }
}