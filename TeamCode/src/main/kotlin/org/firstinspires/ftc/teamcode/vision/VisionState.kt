package org.firstinspires.ftc.teamcode.vision

import arrow.optics.optics
import org.openftc.apriltag.AprilTagDetection

/**
 * Vision-related state.
 *
 * @param aprilTagDetections List of AprilTag detections.
 * @param pixelStackPosition X position of the pixel stack, if any.
 * @param preloadPosition Randomization position for for autonomous.
 */
@optics
data class VisionState(
    val aprilTagDetections: List<AprilTagDetection> = emptyList(),
    val pixelStackPosition: Int? = null,
    val preloadPosition: PreloadDetectionPipeline.RandomizationPosition =
        PreloadDetectionPipeline.RandomizationPosition.RIGHT
) { companion object }
