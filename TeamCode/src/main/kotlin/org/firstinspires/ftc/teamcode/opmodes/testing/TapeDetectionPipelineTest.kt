package org.firstinspires.ftc.teamcode.opmodes.testing

import com.acmerobotics.roadrunner.geometry.Vector2d
import com.outoftheboxrobotics.suspendftc.loopYieldWhile
import com.outoftheboxrobotics.suspendftc.suspendUntil
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import kotlinx.coroutines.launch
import org.firstinspires.ftc.teamcode.RobotState
import org.firstinspires.ftc.teamcode.actions.hardware.currentDrivePose
import org.firstinspires.ftc.teamcode.actions.hardware.dualFixpoint
import org.firstinspires.ftc.teamcode.opmodes.RobotOpMode
import org.firstinspires.ftc.teamcode.util.G
import org.firstinspires.ftc.teamcode.util.cancellationScope
import org.firstinspires.ftc.teamcode.util.set
import org.firstinspires.ftc.teamcode.util.suspendUntilRisingEdge
import org.firstinspires.ftc.teamcode.vision.TapeDetectionPipeline
import org.firstinspires.ftc.teamcode.vision.stackTapePose
import org.firstinspires.ftc.teamcode.visionState
import java.util.concurrent.TimeUnit
import kotlin.math.PI

@Autonomous
class TapeDetectionPipelineTest : RobotOpMode() {
    override suspend fun runSuspendOpMode() {
        val pipeline = TapeDetectionPipeline()

        G.chub.intakeCamera.let {
            it.startCamera(1280, 800)
            streamCamera(it)
            it.setPipeline(pipeline)
            it.showFpsMeterOnViewport(false)
        }

        cancellationScope {
            launch {
                while (true) {
                    suspendUntilRisingEdge { gamepad1.b }
                    pipeline.saveFrameState = true
                    suspendUntil { pipeline.saveFrameState == null }
                    pipeline.saveToDisk()
                    pipeline.saveFrameState = false
                }
            }

            loopYieldWhile({ !gamepad1.a }) {
                telemetry["tape pose"] = G[RobotState.visionState.stackTapePose].toString()
                telemetry["exposure (ms)"] =
                    G.chub.intakeCamera.exposureControl.getExposure(TimeUnit.MILLISECONDS)

                G.ehub.extension.power = -gamepad1.left_stick_y.toDouble()

                if (gamepad1.x) {
                    pipeline.estimate = G[RobotState.visionState.stackTapePose]?.vec()
                }
            }
        }

        val robotTarget = currentDrivePose().vec()
        val intakeTarget = pipeline.estimate ?: G[RobotState.visionState.stackTapePose]?.vec() ?: Vector2d()

        dualFixpoint(
            robotTarget = robotTarget,
            intakeTarget = intakeTarget + (intakeTarget - robotTarget).rotated(PI / 2).let { it / it.norm() }
        )
    }
}