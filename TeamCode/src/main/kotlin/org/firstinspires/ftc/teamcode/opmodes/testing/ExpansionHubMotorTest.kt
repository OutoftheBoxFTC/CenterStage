package org.firstinspires.ftc.teamcode.opmodes.testing

import arrow.core.some
import com.outoftheboxrobotics.suspendftc.loopYieldWhile
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.hardware.devices.ThreadedImuHandler
import org.firstinspires.ftc.teamcode.opmodes.RobotOpMode
import org.firstinspires.ftc.teamcode.util.G

@TeleOp
class ExpansionHubMotorTest : RobotOpMode(
    runMultiThreaded = true,
    imuHandler = ThreadedImuHandler().some()
) {
    override suspend fun runSuspendOpMode() {
        suspendUntilStart()

        loopYieldWhile({ true }) {
            G.ehub.m4.power = if (G.gp1.dpad_down) -1.0 else if (G.gp1.a) 1.0 else 0.0
            G.ehub.m5.power = if (G.gp1.dpad_right) -1.0 else if (G.gp1.b) 1.0 else 0.0
            G.ehub.m6.power = if (G.gp1.dpad_left) -1.0 else if (G.gp1.x) 1.0 else 0.0
            G.ehub.m7.power = if (G.gp1.dpad_up) -1.0 else if (G.gp1.y) 1.0 else 0.0
        }
    }
}