package org.firstinspires.ftc.teamcode.opmodes.testing

import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.opmodes.RobotOpMode
import org.firstinspires.ftc.teamcode.util.G
import org.firstinspires.ftc.teamcode.util.mainLoop

@TeleOp
@Disabled
class HangTest : RobotOpMode() {
    override suspend fun runSuspendOpMode() {
        suspendUntilStart()

        mainLoop {
            G.chub.hang0.power = -gamepad1.left_stick_y.toDouble()
            G.ehub.hang1.power = -gamepad1.left_stick_y.toDouble()
        }
    }

}