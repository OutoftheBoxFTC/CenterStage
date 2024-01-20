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
import org.firstinspires.ftc.teamcode.actions.hardware.followTrajectoryPath
import org.firstinspires.ftc.teamcode.actions.hardware.profileArm
import org.firstinspires.ftc.teamcode.actions.hardware.resetDrivePose
import org.firstinspires.ftc.teamcode.actions.hardware.retractExtension
import org.firstinspires.ftc.teamcode.actions.hardware.scoreOnBackstage
import org.firstinspires.ftc.teamcode.actions.hardware.setClawPos
import org.firstinspires.ftc.teamcode.util.G
import org.firstinspires.ftc.teamcode.util.buildTrajectory
import org.firstinspires.ftc.teamcode.vision.AprilTagDetectionPipeline
import org.firstinspires.ftc.teamcode.vision.PreloadDetectionPipeline.RandomizationPosition
import org.firstinspires.ftc.teamcode.vision.preloadPosition
import org.firstinspires.ftc.teamcode.visionState
import kotlin.math.PI

/**
 * Cycling auto using the center-most lane.
 */
abstract class PreloadBackstageAuto(isBlue: Boolean) : AutonOpMode(isBlue) {
    abstract val rightPreloadFloor: Pose2d
    abstract val frontPreloadFloor: Pose2d
    abstract val leftPreloadFloor: Pose2d

    abstract val preOuttakePreload: Pose2d

    abstract val preOuttake: Pose2d
    abstract val centerOuttakeEstimate: Pose2d

    abstract val parkPos: Pose2d

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

        val rightPreloadOuttakeTraj = floorOuttakeTrajectory(rightPreloadFloor, preOuttake)
        val centerPreloadOuttakeTraj = floorOuttakeTrajectory(frontPreloadFloor, preOuttake)
        val leftPreloadOuttakeTraj = floorOuttakeTrajectory(leftPreloadFloor, preOuttake)

        val parkTrajectory = buildTrajectory(preOuttake) {
            lineToConstantHeading(parkPos.vec())
        }


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

        breakpoint()

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

        breakpoint()

        // Score on backboard
        profileArm(ArmPosition.OUTTAKE)
        // TODO
        scoreOnBackstage(centerOuttakeEstimate, randomizationPos)
        suspendFor(200)

        breakpoint()

        // Drive to pre-intake position
        coroutineScope {
            launch { profileArm(ArmPosition.NEUTRAL) }

            followTrajectoryPath(parkTrajectory)
        }

        breakpoint()

        suspendFor(5000)
    }
}

@Autonomous
class BluePreloadBackstageAuto : PreloadBackstageAuto(true) {
    override val rightPreloadFloor = Pose2d(-18.523, 5.557, 5.781)
    override val frontPreloadFloor = Pose2d(-19.315, 0.427, 6.263)
    override val leftPreloadFloor = Pose2d(-19.151, -1.784, 0.394)

    override val preOuttakePreload = Pose2d(-13.497, -21.995, PI /2)

    override val preOuttake = Pose2d(-25.294, -23.579, PI / 2)
    override val centerOuttakeEstimate = Pose2d(-26.672, -27.819, PI / 2)

    override val parkPos = Pose2d(-4.0, -30.0, PI / 2)
}

@Autonomous
class RedPreloadBackstageAuto : PreloadBackstageAuto(false) {
    override val rightPreloadFloor = Pose2d(-20.378, 1.137, 5.869)
    override val frontPreloadFloor = Pose2d(-19.243, -1.822, 6.273)
    override val leftPreloadFloor = Pose2d(-18.578, -3.857, 0.584)

    override val preOuttakePreload = Pose2d(-6.86, 21.883, -PI / 2)

    override val preOuttake = Pose2d(-26.041, 24.664, -PI / 2)
    override val centerOuttakeEstimate = Pose2d(-28.480, 27.255, -PI / 2)

    override val parkPos = Pose2d(-4.0, 30.0, -PI / 2)
}
