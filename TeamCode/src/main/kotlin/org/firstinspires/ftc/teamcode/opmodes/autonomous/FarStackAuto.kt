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
import org.firstinspires.ftc.teamcode.util.G
import org.firstinspires.ftc.teamcode.util.buildTrajectory
import org.firstinspires.ftc.teamcode.vision.PreloadDetectionPipeline
import org.firstinspires.ftc.teamcode.vision.preloadPosition
import org.firstinspires.ftc.teamcode.visionState
import kotlin.math.PI

abstract class FarStackAuto : AutonOpMode() {
    abstract val rightPreloadFloor: Pose2d
    abstract val frontPreloadFloor: Pose2d
    abstract val leftPreloadFloor: Pose2d

    override suspend fun runSuspendOpMode() = coroutineScope {
        val preloadRightTrajectory = buildTrajectory(Pose2d()) {
            setReversed(true)
            splineTo(rightPreloadFloor.vec(), rightPreloadFloor.heading + PI)
        }

        val preloadCenterTrajectory = buildTrajectory(Pose2d()) {
            setReversed(true)
            splineTo(frontPreloadFloor.vec(), frontPreloadFloor.heading + PI)
        }

        val preloadLeftTrajectory = buildTrajectory(Pose2d()) {
            setReversed(true)
            splineTo(leftPreloadFloor.vec(), leftPreloadFloor.heading + PI)
        }

        launch {
            runAutonInit()
        }.let {
            suspendUntilStart()
            it.cancelAndJoin()
        }

        resetDrivePose(Pose2d())
        setExtensionHold()
        closeClaws()

        profileArm(ArmPosition.FLOOR)

        when (G[RobotState.visionState.preloadPosition]) {
            PreloadDetectionPipeline.RandomizationPosition.RIGHT -> preloadRightTrajectory
            PreloadDetectionPipeline.RandomizationPosition.LEFT -> preloadLeftTrajectory
            PreloadDetectionPipeline.RandomizationPosition.CENTER -> preloadCenterTrajectory
        }.let {
            followTrajectoryFixpoint(it)
        }

        suspendFor(1000)
        setClawPos(ClawPosition.BLACK_OPEN)
        suspendFor(5000)
    }
}

@Autonomous
class BlueFarStackAuto : FarStackAuto() {
    override val rightPreloadFloor = Pose2d(-18.523, 5.557, 5.781)
    override val frontPreloadFloor = Pose2d(-19.315, 0.427, 6.263)
    override val leftPreloadFloor = Pose2d(-19.151, -1.784, 0.394)

    val preOuttakePreload = Pose2d(-8.731, -24.850, 1.566)
    val outtake = Pose2d(-26.771, -26.772, PI / 2)
}

@Autonomous
class RedFarStackAuto : FarStackAuto() {
    override val rightPreloadFloor = Pose2d(-20.378, 1.137, 5.869)
    override val frontPreloadFloor = Pose2d(-21.243, -1.822, 6.273)
    override val leftPreloadFloor = Pose2d(-20.272, -4.788, 0.584)
}
