package org.firstinspires.ftc.teamcode.opmodes.testing

import arrow.core.some
import com.outoftheboxrobotics.suspendftc.loopYieldWhile
import com.outoftheboxrobotics.suspendftc.suspendFor
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import kotlinx.coroutines.coroutineScope
import kotlinx.coroutines.launch
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import org.firstinspires.ftc.teamcode.hardware.devices.ThreadedImuHandler
import org.firstinspires.ftc.teamcode.opmodes.RobotOpMode
import org.firstinspires.ftc.teamcode.util.G
import org.firstinspires.ftc.teamcode.util.set

@TeleOp
class ExtensionTest : RobotOpMode(
    runMultiThreaded = true,
    imuHandler = ThreadedImuHandler().some()
) {
    override suspend fun runSuspendOpMode() {
        suspendUntilStart()

        coroutineScope {
            launch {
                loopYieldWhile({ true }) {
                    suspendFor(50)
                    G.ehub.readCurrents()
                }
            }

            loopYieldWhile({ true }) {
                G.ehub.setExtensionPower(-gamepad1.left_stick_y.toDouble())

                telemetry["encoder pos"] = G.ehub.m7.currentPosition
                telemetry["encoder vel"] = G.ehub.m7.velocity
                telemetry["current"] = G.ehub.run {
                    leftExtension.getCurrent(CurrentUnit.AMPS) + rightExtension.getCurrent(CurrentUnit.AMPS)
                }
            }
        }
    }
}