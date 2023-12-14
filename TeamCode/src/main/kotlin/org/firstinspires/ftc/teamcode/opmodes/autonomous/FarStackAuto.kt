package org.firstinspires.ftc.teamcode.opmodes.autonomous

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.outoftheboxrobotics.suspendftc.suspendFor
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.actions.hardware.followTrajectoryFixpoint
import org.firstinspires.ftc.teamcode.actions.hardware.resetDrivePose
import org.firstinspires.ftc.teamcode.opmodes.RobotOpMode
import org.firstinspires.ftc.teamcode.util.buildTrajectory
import kotlin.math.PI

abstract class FarStackAuto : RobotOpMode() {
    private enum class RandomizationPosition {
        LEFT,
        CENTER,
        RIGHT
    }

    abstract val rightPreloadFloor: Pose2d
    abstract val frontPreloadFloor: Pose2d
    abstract val leftPreloadFloor: Pose2d

    override suspend fun runSuspendOpMode() {
        val preloadTarget = RandomizationPosition.LEFT

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

        suspendUntilStart()
        resetDrivePose(Pose2d())

        when (preloadTarget) {
            RandomizationPosition.RIGHT -> preloadRightTrajectory
            RandomizationPosition.LEFT -> preloadLeftTrajectory
            RandomizationPosition.CENTER -> preloadCenterTrajectory
        }.let {
            followTrajectoryFixpoint(it)
        }

        suspendFor(5000)
    }
}

@Autonomous
class BlueFarStackAuto : FarStackAuto() {
    override val rightPreloadFloor = Pose2d(-18.523, 5.557, 5.781)
    override val frontPreloadFloor = Pose2d(-19.315, 0.427, 6.263)
    override val leftPreloadFloor = Pose2d(-19.151, -1.784, 0.394)

    val preOuttakePreload = Pose2d(-8.731, -24.850, 1.566)
}
