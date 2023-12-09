package org.firstinspires.ftc.teamcode.vision

import arrow.optics.optics
import org.openftc.apriltag.AprilTagDetection

@optics
data class VisionState(
    val aprilTagDetections: List<AprilTagDetection> = emptyList(),
    val pixelStackPosition: Int? = null
) { companion object }
