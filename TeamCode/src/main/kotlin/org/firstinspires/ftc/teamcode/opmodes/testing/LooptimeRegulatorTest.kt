package org.firstinspires.ftc.teamcode.opmodes.testing

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.util.ElapsedTime
import kotlinx.coroutines.coroutineScope
import kotlinx.coroutines.launch
import org.firstinspires.ftc.teamcode.opmodes.RobotOpMode
import org.firstinspires.ftc.teamcode.util.mainLoop
import org.firstinspires.ftc.teamcode.util.set

@TeleOp
class LooptimeRegulatorTest : RobotOpMode() {
    override suspend fun runSuspendOpMode() {
        val timer = ElapsedTime()

        var hz = 0

        coroutineScope {
            launch {
                mainLoop(30) {
                    hz = (1 / timer.seconds()).toInt()
                    timer.reset()
                }
            }

            mainLoop {
                telemetry["hz"] = hz
            }
        }
    }
}