package org.firstinspires.ftc.teamcode.vision

import arrow.optics.optics
import org.openftc.apriltag.AprilTagDetection

@optics
data class VisionState(
    val aprilTagDetections: List<AprilTagDetection>
) { companion object }
