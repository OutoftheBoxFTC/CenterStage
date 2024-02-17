package org.firstinspires.ftc.teamcode.actions.hardware

import arrow.core.merge
import arrow.core.toNonEmptyListOrNull
import arrow.fx.coroutines.raceN
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator
import com.acmerobotics.roadrunner.profile.MotionState
import com.outoftheboxrobotics.suspendftc.suspendFor
import com.outoftheboxrobotics.suspendftc.suspendUntil
import com.qualcomm.robotcore.util.ElapsedTime
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
import org.firstinspires.ftc.teamcode.RobotState
import org.firstinspires.ftc.teamcode.actions.controllers.PidCoefs
import org.firstinspires.ftc.teamcode.actions.controllers.runPidController
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
    finalLiftPos: Int? = null,
    liftEnd: Boolean = true
) = coroutineScope {
    retractLift()

    openClaws()
    setTiltPosition(IntakeTiltPosition.TRANSFER_FLAT)
    suspendFor(100)

    G.ehub.intakeRoller.power = -0.8

    profileArm(ArmPosition.TRANSFER)
    retractExtension()

    val profile = MotionProfileGenerator.generateSimpleMotionProfile(
        MotionState(G.ehub.intakeTilt.position, 0.0),
        MotionState(IntakeTiltPosition.PRE_TRANSFER.pos, 0.0),
        6.0,
        10.0
    )

    val timer = ElapsedTime()

    suspendUntil {
        val t = timer.seconds()
        setTiltPosition(profile[t].x)
        t >= profile.duration()
    }

    suspendFor(100)

    setTiltPosition(IntakeTiltPosition.TRANSFER)

    suspendFor(600)

    G.ehub.intakeRoller.power = 0.0

    closeClaws()
    suspendFor(100)

    setTiltPosition(IntakeTiltPosition.POST_TRANSFER)
    G.ehub.intakeRoller.power = -0.8
    suspendFor(50)
    G.ehub.intakeRoller.power = 0.0


    liftUpTo(LiftConfig.transferHeightMin)

    val armJob = launch {
        profileArm(finalArmPos)
    }

    launch {
        if (finalLiftPos != null) liftUpTo(finalLiftPos)
        else {
            armJob.join()
            if (liftEnd) retractLift()
        }
    }

    setTiltPosition(IntakeTiltPosition.HIGH)
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
                backstagePos.vec() + hvec * 17.0 + hvec.rotated(PI / 2) * when (target) {
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

    launchFixpoint(targetPose) to targetPose
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
            suspendFor(1000)
        },
        {
            suspendUntil { currentDrivePose().vec().distTo(target.vec()) < 1.0 }
            suspendFor(200)
        }
    )

    openClaws()
    job.cancelAndJoin()

    return launchFixpoint(returnPos)
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