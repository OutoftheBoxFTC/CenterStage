package org.firstinspires.ftc.teamcode.opmodes.testing

import com.outoftheboxrobotics.suspendftc.suspendUntil
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import kotlinx.coroutines.coroutineScope
import kotlinx.coroutines.launch
import org.firstinspires.ftc.teamcode.actions.hardware.retractExtension
import org.firstinspires.ftc.teamcode.opmodes.RobotOpMode
import org.firstinspires.ftc.teamcode.util.G
import org.firstinspires.ftc.teamcode.util.mainLoop
import org.firstinspires.ftc.teamcode.util.set

@TeleOp
class ExtensionRetractTest : RobotOpMode() {
    override suspend fun runSuspendOpMode(): Unit = coroutineScope {
        suspendUntilStart()

        launch {
            mainLoop {
                telemetry["limit switch"] = G.chub.extensionLimitSwitch
            }
        }

        while (true) {
            suspendUntil { gamepad1.y }
            retractExtension()
        }
    }
}