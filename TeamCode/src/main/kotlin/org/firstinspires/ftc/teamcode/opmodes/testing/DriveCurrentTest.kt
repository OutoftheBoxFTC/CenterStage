package org.firstinspires.ftc.teamcode.opmodes.testing

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import kotlinx.coroutines.coroutineScope
import kotlinx.coroutines.launch
import org.firstinspires.ftc.teamcode.RobotState
import org.firstinspires.ftc.teamcode.actions.hardware.runFieldCentricDrive
import org.firstinspires.ftc.teamcode.actions.hardware.setExtensionPower
import org.firstinspires.ftc.teamcode.driveLooper
import org.firstinspires.ftc.teamcode.opmodes.RobotOpMode
import org.firstinspires.ftc.teamcode.util.C
import org.firstinspires.ftc.teamcode.util.G
import org.firstinspires.ftc.teamcode.util.mainLoop

@TeleOp
class DriveCurrentTest : RobotOpMode() {
    override suspend fun runSuspendOpMode() {
        suspendUntilStart()

        coroutineScope {
            G[RobotState.driveLooper].scheduleCoroutine {
                mainLoop {
                    G.chub.readCurrents()
                }
            }

            launch {
                mainLoop {
                    setExtensionPower(-gamepad1.right_stick_y.toDouble())
                    G.ehub.intakeWheel.power = C.driveTurn

                    G.ehub.readCurrents()
                }
            }

            runFieldCentricDrive()
        }
    }
}