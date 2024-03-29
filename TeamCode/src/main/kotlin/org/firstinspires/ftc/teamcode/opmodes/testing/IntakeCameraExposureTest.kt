package org.firstinspires.ftc.teamcode.opmodes.testing

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import kotlinx.coroutines.coroutineScope
import kotlinx.coroutines.launch
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl
import org.firstinspires.ftc.teamcode.opmodes.RobotOpMode
import org.firstinspires.ftc.teamcode.util.G
import org.firstinspires.ftc.teamcode.util.mainLoop
import org.firstinspires.ftc.teamcode.util.set
import org.firstinspires.ftc.teamcode.util.suspendUntilRisingEdge
import org.firstinspires.ftc.teamcode.vision.TapeDetectionPipeline
import java.util.concurrent.TimeUnit

@TeleOp
@Config
class IntakeCameraExposureTest : RobotOpMode() {
    companion object {
        @JvmField var exposure = 1
    }

    override suspend fun runSuspendOpMode() {
        val camera = G.chub.intakeCamera

        camera.run {
            startCamera(1280, 800)
            streamCamera(this@run)
            setPipeline(TapeDetectionPipeline())
        }

        suspendUntilStart()

        coroutineScope {
            launch {
                while (true) {
                    suspendUntilRisingEdge { gamepad1.x }

                    camera.exposureControl.let {
                        it.mode = ExposureControl.Mode.Manual
                        it.setExposure(exposure.toLong(), TimeUnit.MILLISECONDS)
                    }
                }
            }

            mainLoop {
                val control = camera.exposureControl

                telemetry["exposure"] = control.getExposure(TimeUnit.MILLISECONDS)
                telemetry["min exposure"] = control.getMinExposure(TimeUnit.MILLISECONDS)
                telemetry["max exposure"] = control.getMaxExposure(TimeUnit.MILLISECONDS)
            }
        }
    }
}