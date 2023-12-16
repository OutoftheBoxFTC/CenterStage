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
import org.firstinspires.ftc.teamcode.actions.hardware.closeClaws
import org.firstinspires.ftc.teamcode.actions.hardware.currentDrivePose
import org.firstinspires.ftc.teamcode.actions.hardware.followTrajectoryFixpoint
import org.firstinspires.ftc.teamcode.actions.hardware.launchFixpoint
import org.firstinspires.ftc.teamcode.actions.hardware.openClaws
import org.firstinspires.ftc.teamcode.actions.hardware.profileArm
import org.firstinspires.ftc.teamcode.actions.hardware.resetDrivePose
import org.firstinspires.ftc.teamcode.actions.hardware.setClawPos
import org.firstinspires.ftc.teamcode.actions.hardware.setExtensionHold
import org.firstinspires.ftc.teamcode.util.G
import org.firstinspires.ftc.teamcode.util.buildTrajectory
import org.firstinspires.ftc.teamcode.util.mapState
import org.firstinspires.ftc.teamcode.vision.AprilTagDetectionPipeline
import org.firstinspires.ftc.teamcode.vision.PreloadDetectionPipeline
import org.firstinspires.ftc.teamcode.vision.preloadPosition
import org.firstinspires.ftc.teamcode.visionState
import kotlin.math.PI

abstract class FarStackAuto(isBlue: Boolean) : AutonOpMode(isBlue) {
    abstract val rightPreloadFloor: Pose2d
    abstract val frontPreloadFloor: Pose2d
    abstract val leftPreloadFloor: Pose2d

    abstract val preOuttakePreload: Pose2d

    abstract val preOuttakeLeft: Pose2d
    abstract val preOuttakeCenter: Pose2d
    abstract val preOuttakeRight: Pose2d

    override suspend fun runSuspendOpMode() = coroutineScope {
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

        launch {
            runAutonInit()
        }.let {
            suspendUntilStart()
            it.cancelAndJoin()
        }

        val randomizationPos = G[RobotState.visionState.preloadPosition]

        G.chub.outtakeCamera.setPipeline(AprilTagDetectionPipeline.outtakePipeline())

        resetDrivePose(Pose2d())
        setExtensionHold()
        closeClaws()

        profileArm(ArmPosition.FLOOR)

        when (randomizationPos) {
            PreloadDetectionPipeline.RandomizationPosition.RIGHT -> preloadRightTrajectory
            PreloadDetectionPipeline.RandomizationPosition.LEFT -> preloadLeftTrajectory
            PreloadDetectionPipeline.RandomizationPosition.CENTER -> preloadCenterTrajectory
        }.let {
            followTrajectoryFixpoint(it)
        }

        suspendFor(400)
        setClawPos(ClawPosition.RED_OPEN)
        suspendFor(100)

        coroutineScope {
            launch { profileArm(ArmPosition.NEUTRAL) }

            when (randomizationPos) {
                PreloadDetectionPipeline.RandomizationPosition.LEFT -> leftPreloadOuttakeTraj
                PreloadDetectionPipeline.RandomizationPosition.CENTER -> centerPreloadOuttakeTraj
                PreloadDetectionPipeline.RandomizationPosition.RIGHT -> rightPreloadOuttakeTraj
            }.let {
                followTrajectoryFixpoint(it)
            }
        }

        profileArm(ArmPosition.OUTTAKE)
        outtakeFixpoint()
        suspendFor(400)
        openClaws()
        suspendFor(200)

        when (randomizationPos) {
            PreloadDetectionPipeline.RandomizationPosition.LEFT -> preOuttakeLeft
            PreloadDetectionPipeline.RandomizationPosition.CENTER -> preOuttakeCenter
            PreloadDetectionPipeline.RandomizationPosition.RIGHT -> preOuttakeRight
        }.let { launchFixpoint(it) }

        suspendFor(5000)
    }

    private suspend fun outtakeFixpoint(): Job = coroutineScope {
        val targetPose = raceN(
            coroutineContext,
            {
                val poses = G.robotState
                    .mapState { it.visionState.aprilTagDetections }
                    .filter { it.isNotEmpty() }
                    .withIndex()
                    .first { it.index > 0 }
                    .value
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

                Pose2d(
                    poses.map { it.x }.average(),
                    poses.map { it.y }.average(),
                    poses.map { it.heading }.average()
                )
            },
            {
                suspendFor(500)
                val pose = currentDrivePose()

                Pose2d(
                    pose.x,
                    pose.y + 4.0 * if (isBlue) -1.0 else 1.0,
                    pose.heading
                )
            }
        ).merge()

        launchFixpoint(targetPose)
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
}

@Autonomous
class RedFarStackAuto : FarStackAuto(false) {
    override val rightPreloadFloor = Pose2d(-20.378, 1.137, 5.869)
    override val frontPreloadFloor = Pose2d(-19.243, -1.822, 6.273)
    override val leftPreloadFloor = Pose2d(-18.272, -4.788, 0.584)

    override val preOuttakePreload = Pose2d(-6.86, 21.883, -PI / 2)

    override val preOuttakeLeft = Pose2d(-32.979, 24.664, -PI / 2)
    override val preOuttakeCenter = Pose2d(-26.041, 24.664, -PI / 2)
    override val preOuttakeRight = Pose2d(-19.400, 24.664, -PI / 2)
}
