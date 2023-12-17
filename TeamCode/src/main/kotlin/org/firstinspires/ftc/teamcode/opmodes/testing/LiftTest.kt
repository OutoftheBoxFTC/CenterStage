package org.firstinspires.ftc.teamcode.opmodes.testing

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.opmodes.RobotOpMode
import org.firstinspires.ftc.teamcode.util.G
import org.firstinspires.ftc.teamcode.util.mainLoop

@TeleOp
@Config
class LiftTest : RobotOpMode() {
    companion object {
        @JvmField var liftPower = 0.0
    }

    override suspend fun runSuspendOpMode() {
        suspendUntilStart()

        mainLoop {
            G.ehub.outtakeLift.power = liftPower
        }
    }
}