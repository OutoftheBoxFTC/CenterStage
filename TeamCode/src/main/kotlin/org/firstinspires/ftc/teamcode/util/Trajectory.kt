package org.firstinspires.ftc.teamcode.util

import com.acmerobotics.roadrunner.geometry.Pose2d
import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceBuilder

/**
 * DSL-esque trajectory sequence builder.
 *
 * @param startPose The starting pose of the robot.
 * @param block The [TrajectorySequenceBuilder] block.
 */
fun buildTrajectory(
    startPose: Pose2d,
    block: TrajectorySequenceBuilder.() -> Unit
): TrajectorySequence = SampleMecanumDrive.trajectorySequenceBuilder(startPose)
    .apply(block)
    .build()

// Convenience functions for setting velocity and acceleration constraints
// on a trajectory sequence builder
fun TrajectorySequenceBuilder.setAccelConstraint(maxAccel: Double): TrajectorySequenceBuilder =
    setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(maxAccel))

fun TrajectorySequenceBuilder.setVelConstraint(maxVel: Double): TrajectorySequenceBuilder =
    setVelConstraint(
        SampleMecanumDrive.getVelocityConstraint(
            maxVel,
            DriveConstants.MAX_ANG_VEL,
            DriveConstants.TRACK_WIDTH
        )
    )