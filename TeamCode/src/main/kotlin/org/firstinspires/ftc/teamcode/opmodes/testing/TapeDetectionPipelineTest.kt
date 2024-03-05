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

@Autonomous
class TapeDetectionPipelineTest : RobotOpMode() {
    override suspend fun runSuspendOpMode() {
        G.chub.intakeCamera.let {
            it.startCamera(1200, 800)
            streamCamera(it)
            it.setPipeline(TapeDetectionPipeline())
        }

        mainLoop {
            telemetry["tape pose"] = G[RobotState.visionState.stackTapePose].toString()
        }
    }
}