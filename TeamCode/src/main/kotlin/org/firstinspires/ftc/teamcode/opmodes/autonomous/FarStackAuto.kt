package org.firstinspires.ftc.teamcode.opmodes.autonomous

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.outoftheboxrobotics.suspendftc.suspendFor
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import kotlinx.coroutines.cancelAndJoin
import kotlinx.coroutines.coroutineScope
import kotlinx.coroutines.launch
import org.firstinspires.ftc.teamcode.RobotState
import org.firstinspires.ftc.teamcode.actions.hardware.ArmPosition
import org.firstinspires.ftc.teamcode.actions.hardware.ClawPosition
import org.firstinspires.ftc.teamcode.actions.hardware.closeClaws
import org.firstinspires.ftc.teamcode.actions.hardware.followTrajectoryFixpoint
import org.firstinspires.ftc.teamcode.actions.hardware.profileArm
import org.firstinspires.ftc.teamcode.actions.hardware.resetDrivePose
import org.firstinspires.ftc.teamcode.actions.hardware.setClawPos
import org.firstinspires.ftc.teamcode.actions.hardware.setExtensionHold
import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants
import org.firstinspires.ftc.teamcode.util.G
import org.firstinspires.ftc.teamcode.util.buildTrajectory
import org.firstinspires.ftc.teamcode.util.mainLoop
import org.firstinspires.ftc.teamcode.util.setAccelConstraint
import org.firstinspires.ftc.teamcode.vision.AprilTagDetectionPipeline
import org.firstinspires.ftc.teamcode.vision.PreloadDetectionPipeline
import org.firstinspires.ftc.teamcode.vision.aprilTagDetections
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
            setAccelConstraint(DriveConstants.MAX_ACCEL / 2)
            splineTo(target.vec(), target.heading + PI)
        }

        val preloadRightTrajectory = preloadFloorTrajectory(rightPreloadFloor)
        val preloadCenterTrajectory = preloadFloorTrajectory(frontPreloadFloor)
        val preloadLeftTrajectory = preloadFloorTrajectory(leftPreloadFloor)

        fun floorOuttakeTrajectory(start: Pose2d, dest: Pose2d) = buildTrajectory(start) {
            splineToSplineHeading(preOuttakePreload, -PI)
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
            launch { profileArm(ArmPosition.OUTTAKE) }

            when (randomizationPos) {
                PreloadDetectionPipeline.RandomizationPosition.LEFT -> leftPreloadOuttakeTraj
                PreloadDetectionPipeline.RandomizationPosition.CENTER -> centerPreloadOuttakeTraj
                PreloadDetectionPipeline.RandomizationPosition.RIGHT -> rightPreloadOuttakeTraj
            }.let {
                followTrajectoryFixpoint(it)
            }
        }

        mainLoop {
            telemetry.addLine(
                G[RobotState.visionState.aprilTagDetections].map { it.pose.x }.joinToString()
            )
        }
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

    override val preOuttakeLeft = Pose2d(-31.979, 24.664, -PI / 2)
    override val preOuttakeCenter = Pose2d(-26.041, 24.664, -PI / 2)
    override val preOuttakeRight = Pose2d(-20.400, 24.664, -PI / 2)


}
