package org.firstinspires.ftc.teamcode.opmodes.autonomous

import arrow.core.merge
import arrow.fx.coroutines.raceN
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.outoftheboxrobotics.suspendftc.suspendFor
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import kotlinx.coroutines.Job
import kotlinx.coroutines.cancelAndJoin
import kotlinx.coroutines.coroutineScope
import kotlinx.coroutines.flow.filter
import kotlinx.coroutines.flow.first
import kotlinx.coroutines.flow.withIndex
import kotlinx.coroutines.launch
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference
import org.firstinspires.ftc.robotcore.external.navigation.Orientation
import org.firstinspires.ftc.teamcode.RobotState
import org.firstinspires.ftc.teamcode.actions.hardware.ArmPosition
import org.firstinspires.ftc.teamcode.actions.hardware.ClawPosition
import org.firstinspires.ftc.teamcode.actions.hardware.IntakeTiltPosition
import org.firstinspires.ftc.teamcode.actions.hardware.closeClaws
import org.firstinspires.ftc.teamcode.actions.hardware.currentDrivePose
import org.firstinspires.ftc.teamcode.actions.hardware.followLinePath
import org.firstinspires.ftc.teamcode.actions.hardware.followTrajectory
import org.firstinspires.ftc.teamcode.actions.hardware.followTrajectoryFixpoint
import org.firstinspires.ftc.teamcode.actions.hardware.launchExtensionPid
import org.firstinspires.ftc.teamcode.actions.hardware.launchFixpoint
import org.firstinspires.ftc.teamcode.actions.hardware.openClaws
import org.firstinspires.ftc.teamcode.actions.hardware.profileArm
import org.firstinspires.ftc.teamcode.actions.hardware.resetDrivePose
import org.firstinspires.ftc.teamcode.actions.hardware.resetExtensionLength
import org.firstinspires.ftc.teamcode.actions.hardware.retractExtension
import org.firstinspires.ftc.teamcode.actions.hardware.runExtensionTo
import org.firstinspires.ftc.teamcode.actions.hardware.setClawPos
import org.firstinspires.ftc.teamcode.actions.hardware.setExtensionHold
import org.firstinspires.ftc.teamcode.actions.hardware.setTiltPosition
import org.firstinspires.ftc.teamcode.actions.hardware.swoop
import org.firstinspires.ftc.teamcode.util.G
import org.firstinspires.ftc.teamcode.util.buildTrajectory
import org.firstinspires.ftc.teamcode.util.deg
import org.firstinspires.ftc.teamcode.util.mapState
import org.firstinspires.ftc.teamcode.vision.AprilTagDetectionPipeline
import org.firstinspires.ftc.teamcode.vision.PreloadDetectionPipeline
import org.firstinspires.ftc.teamcode.vision.PreloadDetectionPipeline.RandomizationPosition
import org.firstinspires.ftc.teamcode.vision.preloadPosition
import org.firstinspires.ftc.teamcode.visionState
import kotlin.math.PI

/**
 * Cycling auto using the center-most lane.
 */
abstract class FarStackAuto(isBlue: Boolean) : AutonOpMode(isBlue) {
    abstract val rightPreloadFloor: Pose2d
    abstract val frontPreloadFloor: Pose2d
    abstract val leftPreloadFloor: Pose2d

    abstract val preOuttakePreload: Pose2d

    abstract val preOuttakeLeft: Pose2d
    abstract val preOuttakeCenter: Pose2d
    abstract val preOuttakeRight: Pose2d

    abstract val preIntakePos: Pose2d
    abstract val intakePos: Pose2d

    open val preExtensionLength = 960
    open val fullExtensionLength = 1180

    override suspend fun runSuspendOpMode(): Unit = coroutineScope {
        fun preloadFloorTrajectory(target: Pose2d) = buildTrajectory(Pose2d()) {
            setReversed(true)
            splineTo(target.vec(), target.heading + PI)
        }

        val preloadRightTrajectory = preloadFloorTrajectory(rightPreloadFloor)
        val preloadCenterTrajectory = preloadFloorTrajectory(frontPreloadFloor)
        val preloadLeftTrajectory = preloadFloorTrajectory(leftPreloadFloor)

        fun floorOuttakeTrajectory(start: Pose2d, dest: Pose2d) = buildTrajectory(start) {
            splineToLinearHeading(preOuttakePreload, -PI)
            splineToConstantHeading(dest.vec(), -PI)
        }

        val rightPreloadOuttakeTraj = floorOuttakeTrajectory(rightPreloadFloor, preOuttakeRight)
        val centerPreloadOuttakeTraj = floorOuttakeTrajectory(frontPreloadFloor, preOuttakeCenter)
        val leftPreloadOuttakeTraj = floorOuttakeTrajectory(leftPreloadFloor, preOuttakeLeft)

        fun outtakePreIntakeTrajectory(start: Pose2d, dest: Pose2d) = buildTrajectory(start) {
            lineToConstantHeading(dest.vec())
        }

        val rightOuttakePreIntakeTraj = outtakePreIntakeTrajectory(preOuttakeRight, preIntakePos)
        val centerOuttakePreIntakeTraj = outtakePreIntakeTrajectory(preOuttakeCenter, preIntakePos)
        val leftOuttakePreIntakeTraj = outtakePreIntakeTrajectory(preOuttakeLeft, preIntakePos)


        launch {
            runAutonInit()
        }.let {
            suspendUntilStart()
            it.cancelAndJoin()
        }

        val randomizationPos = G[RobotState.visionState.preloadPosition]

        G.chub.outtakeCamera.setPipeline(AprilTagDetectionPipeline.outtakePipeline())

        resetDrivePose(Pose2d())
        closeClaws()

        // Drive to preload position
        coroutineScope {
            launch { retractExtension() }
            launch { profileArm(ArmPosition.FLOOR) }

            when (randomizationPos) {
                RandomizationPosition.RIGHT -> preloadRightTrajectory
                RandomizationPosition.LEFT -> preloadLeftTrajectory
                RandomizationPosition.CENTER -> preloadCenterTrajectory
            }.let {
                suspendFor(250)
                followTrajectoryFixpoint(it)
            }
        }

        suspendFor(200)
        setClawPos(ClawPosition.RED_OPEN)

        // Drive to backboard to outtake yellow pixel
        coroutineScope {
            launch { profileArm(ArmPosition.NEUTRAL) }

            when (randomizationPos) {
                RandomizationPosition.LEFT -> leftPreloadOuttakeTraj
                RandomizationPosition.CENTER -> centerPreloadOuttakeTraj
                RandomizationPosition.RIGHT -> rightPreloadOuttakeTraj
            }.let {
                followTrajectoryFixpoint(it)
            }
        }

        // Score on backboard
        profileArm(ArmPosition.OUTTAKE)
        outtakeFixpoint()
        suspendFor(400)
        openClaws()
        suspendFor(200)

        // Go back to pre-outtake position
        when (randomizationPos) {
            RandomizationPosition.LEFT -> preOuttakeLeft
            RandomizationPosition.CENTER -> preOuttakeCenter
            RandomizationPosition.RIGHT -> preOuttakeRight
        }.let {
            launchFixpoint(it.copy(
                y = it.y + 1.0 * if (isBlue) -1.0 else -1.0
            ))
        }

        suspendFor(200)

        // Drive to pre-intake position
        coroutineScope {
            launch { profileArm(ArmPosition.NEUTRAL) }

            when (randomizationPos) {
                RandomizationPosition.LEFT -> leftOuttakePreIntakeTraj
                RandomizationPosition.CENTER -> centerOuttakePreIntakeTraj
                RandomizationPosition.RIGHT -> rightOuttakePreIntakeTraj
            }.let { followTrajectoryFixpoint(it) }
        }

        // Park for now
        launchFixpoint(preIntakePos + Pose2d(
            0.0,
            6.0 * if (isBlue) -1.0 else 1.0,
            0.0
        ))


        // In progress cycling auto
//        followLinePath(preIntakePos.vec(), intakePos.vec(), intakePos.heading, stopDist = 6.0)
//        launchFixpoint(intakePos)
//        suspendFor(500)
//        G.chub.intakeWheel.power = 0.5
//        runExtensionTo(preExtensionLength, keepPid = true)
//        G.ehub.intakeRoller.power = -1.0
//        runExtensionTo(fullExtensionLength, keepPid = false)
//        suspendFor(100)
//        runExtensionTo(preExtensionLength, keepPid = false)

        suspendFor(5000)
    }

    /**
     * Launches fixpoint to score position based on apriltag detections.
     */
    private suspend fun outtakeFixpoint(): Job = coroutineScope {
        val targetPose = raceN(
            coroutineContext,
            {
                val poses = G.robotState
                    .mapState { it.visionState.aprilTagDetections }
                    // Wait for next non-empty detection list
                    .filter { it.isNotEmpty() }
                    .withIndex()
                    .first { it.index > 0 }
                    .value
                    // Get target pose
                    .map { it to currentDrivePose() }
                    .map { (detection, currentPose) ->
                        val fwd = detection.pose.z - 8.63
                        val heading = Orientation.getOrientation(
                            detection.pose.R,
                            AxesReference.INTRINSIC,
                            AxesOrder.YXZ,
                            AngleUnit.RADIANS
                        ).firstAngle

                        Pose2d(
                            currentPose.x,
                            currentPose.y + fwd * if (isBlue) -1 else 1,
                            currentPose.heading - heading
                        )
                    }

                // Average from all apriltags
                Pose2d(
                    poses.map { it.x }.average(),
                    poses.map { it.y }.average(),
                    poses.map { it.heading }.average()
                )
            },
            {
                // Fallback if we don't see any apriltags
                suspendFor(500)
                val pose = currentDrivePose()

                Pose2d(
                    pose.x,
                    pose.y + 4.0 * if (isBlue) -1.0 else 1.0,
                    pose.heading
                )
            }
        ).merge()

        launchFixpoint(targetPose, 0.5)
    }
}

@Autonomous
class BlueFarStackAuto : FarStackAuto(true) {
    override val rightPreloadFloor = Pose2d(-18.523, 5.557, 5.781)
    override val frontPreloadFloor = Pose2d(-19.315, 0.427, 6.263)
    override val leftPreloadFloor = Pose2d(-19.151, -1.784, 0.394)

    override val preOuttakePreload = Pose2d(-13.497, -21.995, PI /2)

    override val preOuttakeLeft = Pose2d(-18.626, -23.579, PI / 2)
    override val preOuttakeCenter = Pose2d(-25.294, -23.579, PI / 2)
    override val preOuttakeRight = Pose2d(-31.330, -23.579, PI / 2)

    override val preIntakePos = Pose2d(-49.679, -23.174, PI / 2)
    override val intakePos = Pose2d(-49.679, 6.210, PI / 2)
}

@Autonomous
class RedFarStackAuto : FarStackAuto(false) {
    override val rightPreloadFloor = Pose2d(-20.378, 1.137, 5.869)
    override val frontPreloadFloor = Pose2d(-19.243, -1.822, 6.273)
    override val leftPreloadFloor = Pose2d(-18.272, -4.788, 0.584)

    override val preOuttakePreload = Pose2d(-6.86, 21.883, -PI / 2)

    override val preOuttakeLeft = Pose2d(-32.979, 24.664, -PI / 2)
    override val preOuttakeCenter = Pose2d(-26.041, 24.664, -PI / 2)
    override val preOuttakeRight = Pose2d(-20.400, 24.664, -PI / 2)

    override val preIntakePos = Pose2d(-49.276, 24.719, -PI / 2)
    override val intakePos = Pose2d(-49.276, -5.471, -PI / 2)
}
