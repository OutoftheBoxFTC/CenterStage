package org.firstinspires.ftc.teamcode.opmodes.tuning

import com.outoftheboxrobotics.suspendftc.suspendUntil
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import kotlinx.coroutines.coroutineScope
import kotlinx.coroutines.launch
import org.firstinspires.ftc.teamcode.RobotState
import org.firstinspires.ftc.teamcode.actions.hardware.extendoPose
import org.firstinspires.ftc.teamcode.driveState
import org.firstinspires.ftc.teamcode.opmodes.RobotOpMode
import org.firstinspires.ftc.teamcode.util.G
import org.firstinspires.ftc.teamcode.util.mainLoop
import org.firstinspires.ftc.teamcode.util.set
import org.firstinspires.ftc.teamcode.util.use
import org.firstinspires.ftc.teamcode.vision.TapeDetectionPipeline

@TeleOp
class IntakeCameraPositionTuner : RobotOpMode() {
    override suspend fun runSuspendOpMode() = coroutineScope {
        val pipeline = TapeDetectionPipeline()

        G.chub.intakeCamera.let {
            it.startCamera(1280, 800)
            streamCamera(it)
            it.setPipeline(pipeline)
        }

        launch {
            mainLoop {
                telemetry["relative pose"] = pipeline.cameraPoseEstimate
            }
        }.use {
            suspendUntil { gamepad1.x }
        }

        val cameraRelativePose = pipeline.cameraPoseEstimate
        val startingExtendoPose = G[RobotState.driveState.extendoPose]

        suspendUntil { gamepad1.y }

        val extendoDelta = (G[RobotState.driveState.extendoPose] - startingExtendoPose).vec()
            .rotated(-startingExtendoPose.heading)

        val cameraPos = extendoDelta - cameraRelativePose.vec()

        mainLoop {
            telemetry["Camera Position"] = cameraPos
        }
    }
}