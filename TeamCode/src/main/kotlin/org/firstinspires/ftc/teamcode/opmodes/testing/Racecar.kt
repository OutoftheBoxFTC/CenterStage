package org.firstinspires.ftc.teamcode.opmodes.testing

import com.acmerobotics.dashboard.config.Config
import com.outoftheboxrobotics.suspendftc.suspendUntil
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.util.ElapsedTime
import kotlinx.coroutines.coroutineScope
import kotlinx.coroutines.launch
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import org.firstinspires.ftc.teamcode.opmodes.RobotOpMode
import org.firstinspires.ftc.teamcode.util.G
import org.firstinspires.ftc.teamcode.util.mainLoop
import org.firstinspires.ftc.teamcode.util.set

@TeleOp
@Config
class Racecar : RobotOpMode() {
    companion object {
        @JvmField var targetDist = 1300
    }

    override suspend fun runSuspendOpMode() = coroutineScope {
        launch {
            mainLoop {
                G.ehub.readCurrents()

                telemetry["extension current"] = G.ehub.extension.getCurrent(CurrentUnit.AMPS)
            }
        }

        suspendUntilStart()

        suspendUntil { gamepad1.right_bumper }

        val timer = ElapsedTime()
        val p0 = G.ehub.extension.currentPosition

        G.ehub.extension.power = 1.0

        suspendUntil { !gamepad1.right_bumper || G.ehub.extension.currentPosition - p0 >= targetDist }

        G.ehub.extension.power = 0.0

        val duration = timer.seconds()

        mainLoop {
            telemetry["time"] = duration
        }
    }
}