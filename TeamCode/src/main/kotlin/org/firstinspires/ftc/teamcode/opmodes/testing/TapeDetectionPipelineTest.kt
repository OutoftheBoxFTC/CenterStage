package org.firstinspires.ftc.teamcode.opmodes.testing

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.RobotState
import org.firstinspires.ftc.teamcode.opmodes.RobotOpMode
import org.firstinspires.ftc.teamcode.util.G
import org.firstinspires.ftc.teamcode.util.mainLoop
import org.firstinspires.ftc.teamcode.util.set
import org.firstinspires.ftc.teamcode.vision.TapeDetectionPipeline
import org.firstinspires.ftc.teamcode.vision.stackTapePose
import org.firstinspires.ftc.teamcode.visionState
import java.util.concurrent.TimeUnit

@Autonomous
class TapeDetectionPipelineTest : RobotOpMode() {
    override suspend fun runSuspendOpMode() {
        val pipeline = TapeDetectionPipeline()

        G.chub.intakeCamera.let {
            it.startCamera(1280, 800)
            streamCamera(it)
            it.setPipeline(pipeline)
        }

        mainLoop {
            telemetry["tape pose"] = G[RobotState.visionState.stackTapePose].toString()
            telemetry["exposure (ms)"] = G.chub.intakeCamera.exposureControl.getExposure(TimeUnit.MILLISECONDS)

            if (gamepad1.x) {
                pipeline.estimate = G[RobotState.visionState.stackTapePose]?.vec()
            }
        }
    }
}