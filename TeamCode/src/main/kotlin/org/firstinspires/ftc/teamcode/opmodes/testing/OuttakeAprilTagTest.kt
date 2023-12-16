package org.firstinspires.ftc.teamcode.opmodes.testing

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference
import org.firstinspires.ftc.robotcore.external.navigation.Orientation
import org.firstinspires.ftc.teamcode.RobotState
import org.firstinspires.ftc.teamcode.opmodes.RobotOpMode
import org.firstinspires.ftc.teamcode.util.G
import org.firstinspires.ftc.teamcode.util.mainLoop
import org.firstinspires.ftc.teamcode.vision.AprilTagDetectionPipeline
import org.firstinspires.ftc.teamcode.vision.aprilTagDetections
import org.firstinspires.ftc.teamcode.visionState
import org.openftc.apriltag.AprilTagDetection
import org.openftc.easyopencv.OpenCvCameraRotation

@Autonomous
class OuttakeAprilTagTest : RobotOpMode() {
    override suspend fun runSuspendOpMode() {
        G.chub.outtakeCamera.let {
            it.startCamera(640, 480, OpenCvCameraRotation.UPRIGHT)
            streamCamera(it)
            it.setPipeline(AprilTagDetectionPipeline.outtakePipeline())
        }

        mainLoop {
            val detections = G[RobotState.visionState.aprilTagDetections]

            detections.forEach {
                telemetry.addLine(it.formatted())
            }
        }
    }

    private fun AprilTagDetection.formatted(): String {
        val rot = Orientation.getOrientation(pose.R, AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES)

        return "AprilTagDetection(id=$id, x=${pose.x}, y=${pose.y}, z=${pose.z}, yaw=${rot.firstAngle}, pitch=${rot.secondAngle}"
    }
}