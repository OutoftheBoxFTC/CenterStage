package org.firstinspires.ftc.teamcode.opmodes.testing

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import org.firstinspires.ftc.teamcode.RobotState
import org.firstinspires.ftc.teamcode.opmodes.RobotOpMode
import org.firstinspires.ftc.teamcode.util.G
import org.firstinspires.ftc.teamcode.util.mainLoop
import org.firstinspires.ftc.teamcode.util.set
import org.firstinspires.ftc.teamcode.vision.PreloadDetectionPipeline
import org.firstinspires.ftc.teamcode.vision.preloadPosition
import org.firstinspires.ftc.teamcode.visionState

@Autonomous
@Disabled
class PreloadDetectionTest : RobotOpMode() {
    override suspend fun runSuspendOpMode() {
        val pipeline = PreloadDetectionPipeline()

        G.chub.outtakeCamera.let {
            it.startCamera(640, 480)
            streamCamera(it)
            it.setPipeline(pipeline)
        }

        mainLoop {
            pipeline.run {
                telemetry["Center"] = listOf(mainRedStrength, mainBlueStrength)
                    .map { "%.2f".format(it) }

                telemetry["Side"] = listOf(sideRedStrength, sideBlueStrength)
                    .map { "%.2f".format(it) }
            }

            telemetry["Color"] = if (PreloadDetectionPipeline.isBlue) "Blue" else "Red"
            telemetry["Position"] = G[RobotState.visionState.preloadPosition]

            if (gamepad1.x) pipeline.setBlueBack()
            if (gamepad1.y) pipeline.setRedBack()
            if (gamepad1.a) pipeline.setBlueAud()
            if (gamepad1.b) pipeline.setRedAud()
        }
    }
}