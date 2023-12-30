package org.firstinspires.ftc.teamcode.vision

import arrow.optics.optics
import com.acmerobotics.roadrunner.geometry.Pose2d
import org.openftc.apriltag.AprilTagDetection

/**
 * Vision-related state.
 *
 * @param aprilTagDetections List of AprilTag detections.
 * @param stackTapePose Pose of the middle of the bottom edge of the stack tape.
 * @param preloadPosition Randomization position for for autonomous.
 */
@optics
data class VisionState(
    val aprilTagDetections: List<AprilTagDetection> = emptyList(),
    val stackTapePose: Pose2d? = null,
    val preloadPosition: PreloadDetectionPipeline.RandomizationPosition =
        PreloadDetectionPipeline.RandomizationPosition.RIGHT
) { companion object }
