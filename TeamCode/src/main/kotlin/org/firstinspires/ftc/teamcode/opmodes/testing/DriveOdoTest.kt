package org.firstinspires.ftc.teamcode.opmodes.testing

import com.outoftheboxrobotics.suspendftc.loopYieldWhile
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.G
import org.firstinspires.ftc.teamcode.opmodes.RobotOpMode
import org.firstinspires.ftc.teamcode.set

@TeleOp
class DriveOdoTest : RobotOpMode(runMultiThreaded = true) {
    override suspend fun runSuspendOpMode() {
        suspendUntilStart()

        loopYieldWhile({ true }) {
            with(G.chub) {
                tl.power = if (G.gp1.left_trigger > 0.9) 1.0 else 0.0
                tr.power = if (G.gp1.right_trigger > 0.9) 1.0 else 0.0
                bl.power = if (G.gp1.left_bumper) 1.0 else 0.0
                br.power = if (G.gp1.right_bumper) 1.0 else 0.0

                telemetry["odo right"] = odoRight
                telemetry["odo aux"] = odoAux
                telemetry["odo left"] = odoLeft
            }
        }
    }
}