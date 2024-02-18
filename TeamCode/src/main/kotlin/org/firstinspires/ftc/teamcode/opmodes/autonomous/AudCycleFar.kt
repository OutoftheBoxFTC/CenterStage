package org.firstinspires.ftc.teamcode.opmodes.autonomous

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.outoftheboxrobotics.suspendftc.suspendFor
import com.outoftheboxrobotics.suspendftc.suspendUntil
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.util.ElapsedTime
import kotlinx.coroutines.coroutineScope
import kotlinx.coroutines.launch
import org.firstinspires.ftc.teamcode.RobotState
import org.firstinspires.ftc.teamcode.actions.hardware.ArmPosition
import org.firstinspires.ftc.teamcode.actions.hardware.ClawPosition
import org.firstinspires.ftc.teamcode.actions.hardware.IntakeTiltPosition
import org.firstinspires.ftc.teamcode.actions.hardware.TwistPosition
import org.firstinspires.ftc.teamcode.actions.hardware.closeClaws
import org.firstinspires.ftc.teamcode.actions.hardware.extensionLength
import org.firstinspires.ftc.teamcode.actions.hardware.followTrajectory
import org.firstinspires.ftc.teamcode.actions.hardware.followTrajectoryFixpoint
import org.firstinspires.ftc.teamcode.actions.hardware.intakeTransfer
import org.firstinspires.ftc.teamcode.actions.hardware.profileArm
import org.firstinspires.ftc.teamcode.actions.hardware.resetDrivePose
import org.firstinspires.ftc.teamcode.actions.hardware.retractExtension
import org.firstinspires.ftc.teamcode.actions.hardware.retractLift
import org.firstinspires.ftc.teamcode.actions.hardware.scoreOnBackstage
import org.firstinspires.ftc.teamcode.actions.hardware.setClawPos
import org.firstinspires.ftc.teamcode.actions.hardware.setExtensionPower
import org.firstinspires.ftc.teamcode.actions.hardware.setTiltPosition
import org.firstinspires.ftc.teamcode.actions.hardware.setTwistPosition
import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.util.G
import org.firstinspires.ftc.teamcode.util.buildTrajectory
import org.firstinspires.ftc.teamcode.util.deg
import org.firstinspires.ftc.teamcode.util.use
import org.firstinspires.ftc.teamcode.vision.AprilTagDetectionPipeline
import org.firstinspires.ftc.teamcode.vision.PreloadDetectionPipeline.RandomizationPosition
import org.firstinspires.ftc.teamcode.vision.preloadPosition
import org.firstinspires.ftc.teamcode.visionState
import kotlin.math.PI

abstract class AudCycleFarAuto(isBlue: Boolean) : AutonOpMode(isBlue, true) {
    abstract val prePreloadOuttake: Pose2d

    abstract val rightPreloadFloor: Pose2d
    abstract val frontPreloadFloor: Pose2d
    abstract val leftPreloadFloor: Pose2d

    abstract val preloadIntakePose: Pose2d
    open val preloadExtensionLength = 150

    abstract val preSwoop: Pose2d
    abstract val preOuttake: Pose2d

    abstract val outtakeRightEstimate: Pose2d
    abstract val outtakeCenterEstimate: Pose2d
    abstract val outtakeLeftEstimate: Pose2d

    abstract val park: Pose2d

    override suspend fun runSuspendOpMode() = coroutineScope {
        fun preloadFloorTrajectory(target: Pose2d) = buildTrajectory(Pose2d()) {
            setReversed(true)
            splineTo(prePreloadOuttake.vec(), PI)
            splineToLinearHeading(target, target.heading)
        }

        fun floorIntakeTrajectory(start: Pose2d) = buildTrajectory(start) {
            splineTo(preloadIntakePose.vec(), preloadIntakePose.heading)
        }

        val preloadRightTrajectory = preloadFloorTrajectory(rightPreloadFloor)
        val preloadCenterTrajectory = preloadFloorTrajectory(frontPreloadFloor)
        val preloadLeftTrajectory = preloadFloorTrajectory(leftPreloadFloor)

        val rightPreloadOuttakeTraj = floorIntakeTrajectory(rightPreloadFloor)
        val centerPreloadOuttakeTraj = floorIntakeTrajectory(frontPreloadFloor)
        val leftPreloadOuttakeTraj = floorIntakeTrajectory(leftPreloadFloor)

        val preloadIntakeOuttakeTraj = buildTrajectory(preloadIntakePose) {
            setReversed(true)
            setAccelConstraint(
                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL / 4.0)
            )
            lineTo(preSwoop.vec())
            splineToConstantHeading(preOuttake.vec(), preOuttake.heading + PI)
        }

        val parkTraj = buildTrajectory(preOuttake) {
            setTangent(-PI)
            splineToConstantHeading(park.vec(), park.heading + PI)
        }

        launch {
            runAutonInit()
        }.use {
            suspendUntilStart()
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
        setClawPos(ClawPosition.BLACK_OPEN)

        suspendFor(600)

        // Drive to pre intake
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

        setTiltPosition(IntakeTiltPosition.POS_1)
        G.ehub.intakeRoller.power = -1.0
        suspendFor(1000)

        setExtensionPower(0.7)
        val timer = ElapsedTime()
        suspendUntil { extensionLength() >= preloadExtensionLength || timer.seconds() >= 1.0}
        setExtensionPower(0.0)

        suspendFor(400)

        coroutineScope {
            launch {
                retractExtension()
            }

            followTrajectory(preloadIntakeOuttakeTraj)
        }

        breakpoint()

        intakeTransfer(finalLiftPos = 305)
        setTwistPosition(TwistPosition.POS_3)

        breakpoint()

        scoreOnBackstage(
            when (randomizationPos) {
                RandomizationPosition.LEFT -> outtakeLeftEstimate
                RandomizationPosition.CENTER -> outtakeCenterEstimate
                RandomizationPosition.RIGHT -> outtakeRightEstimate
            },
            randomizationPos
        )

        suspendFor(200)

        breakpoint()

        coroutineScope {
            launch {
                profileArm(ArmPosition.NEUTRAL)
                retractLift()
            }

            followTrajectory(parkTraj)
        }

        breakpoint()

        suspendFor(5000)
    }
}

@Autonomous
class RedAudCycleFar : AudCycleFarAuto(false) {
    override val prePreloadOuttake = Pose2d(-12.0, -7.822, 0.0)

    override val rightPreloadFloor = Pose2d( -37.369, -1.8, 254.916.deg)
    override val frontPreloadFloor = Pose2d(-42.581, -10.893, 251.828.deg)
    override val leftPreloadFloor = Pose2d(-46.531, -9.778, 182.910.deg)

    override val preloadIntakePose = Pose2d(-49.859, -14.914, 270.0.deg)

    override val preSwoop = Pose2d(-52.422, 55.168, 270.0.deg)
    override val preOuttake = Pose2d(-29.142, 75.877, 270.0.deg)

    override val outtakeRightEstimate = Pose2d(-19.918, 78.573, 270.0.deg)
    override val outtakeCenterEstimate = Pose2d(-24.803, 79.088, 270.0.deg)
    override val outtakeLeftEstimate = Pose2d(31.793, 78.825, 270.0.deg)

    override val park = Pose2d(-53.193, 82.901, 270.0.deg)
}

@Autonomous
class BlueAudCycleFar : AudCycleFarAuto(true) {
    override val prePreloadOuttake = Pose2d(-16.376, 8.315, 0.0)

    override val leftPreloadFloor = Pose2d(-34.982, 2.297, 104.607.deg)
    override val frontPreloadFloor = Pose2d(-42.404, 9.904, 109.653.deg)
    override val rightPreloadFloor = Pose2d(-47.941, 13.199, 161.320.deg)

    override val preloadIntakePose = Pose2d(-50.387, 14.434, 90.0.deg)

    override val preSwoop = Pose2d(-51.292, -57.422, 90.0.deg)
    override val preOuttake = Pose2d(-25.634, -75.850,90.0.deg)

    override val outtakeRightEstimate = Pose2d(-31.614, -79.335, 90.0.deg)
    override val outtakeCenterEstimate = Pose2d(26.562, -79.304, 90.0.deg)
    override val outtakeLeftEstimate = Pose2d(-20.305, -79.359, 90.0.deg)

    override val park = Pose2d(-51.311, -81.531, 90.0.deg)
}