package org.firstinspires.ftc.teamcode.opmodes.testing

import com.outoftheboxrobotics.suspendftc.loopYieldWhile
import com.outoftheboxrobotics.suspendftc.suspendFor
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import kotlinx.coroutines.coroutineScope
import kotlinx.coroutines.launch
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import org.firstinspires.ftc.teamcode.opmodes.RobotOpMode
import org.firstinspires.ftc.teamcode.util.G
import org.firstinspires.ftc.teamcode.util.set

@TeleOp
class ExtensionTest : RobotOpMode() {
    override suspend fun runSuspendOpMode() {
        suspendUntilStart()

        coroutineScope {
            launch {
                loopYieldWhile({ true }) {
                    suspendFor(250)
                    G.ehub.readCurrents()
                }
            }

            loopYieldWhile({ true }) {
                G.ehub.extension.power = -gamepad1.left_stick_y.toDouble()

                telemetry["encoder pos"] = G.ehub.extension.currentPosition
                telemetry["encoder vel"] = G.ehub.extension.velocity
                telemetry["current"] = G.ehub.extension.getCurrent(CurrentUnit.AMPS)
            }
        }
    }
}