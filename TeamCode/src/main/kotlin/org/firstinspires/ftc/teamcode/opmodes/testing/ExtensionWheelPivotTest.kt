package org.firstinspires.ftc.teamcode.opmodes.testing

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.opmodes.RobotOpMode
import org.firstinspires.ftc.teamcode.util.G
import org.firstinspires.ftc.teamcode.util.mainLoop
import kotlin.math.sign

@Autonomous
class ExtensionWheelPivotTest : RobotOpMode() {
    override suspend fun runSuspendOpMode() {
        suspendUntilStart()

        mainLoop {
            G.chub.run {
                intakeWheel.power = -sign(odoIntake.currentPosition.toDouble())
            }
        }
    }
}