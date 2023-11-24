package org.firstinspires.ftc.teamcode.opmodes.testing

import arrow.core.nonEmptyListOf
import arrow.core.some
import com.outoftheboxrobotics.suspendftc.loopYieldWhile
import com.outoftheboxrobotics.suspendftc.suspendUntil
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import kotlinx.coroutines.coroutineScope
import kotlinx.coroutines.launch
import org.firstinspires.ftc.teamcode.actions.hardware.runFieldCentricDrive
import org.firstinspires.ftc.teamcode.command.Subsystem
import org.firstinspires.ftc.teamcode.hardware.devices.ThreadedImuHandler
import org.firstinspires.ftc.teamcode.opmodes.RobotOpMode
import org.firstinspires.ftc.teamcode.statemachine.runStateMachine
import org.firstinspires.ftc.teamcode.util.FS
import org.firstinspires.ftc.teamcode.util.G
import org.firstinspires.ftc.teamcode.util.launchCommand
import org.firstinspires.ftc.teamcode.util.set

@TeleOp
class DriveTest : RobotOpMode(
    runMultiThreaded = true,
    imuHandler = ThreadedImuHandler().some()
) {
    private val joystickDrive: FS = FS {
        G.cmd.launchCommand(nonEmptyListOf(Subsystem.DRIVETRAIN)) {
            launch {
                loopYieldWhile({ true }) {
                    telemetry["Current State"] = "Joystick Drive"
                }
            }

            runFieldCentricDrive()
        }

        suspendUntil { G.gp1.a }
        buttonPower
    }

    private val buttonPower: FS = FS {
        G.cmd.launchCommand(nonEmptyListOf(Subsystem.DRIVETRAIN)) {
            loopYieldWhile({ true }) {
                with(G.chub) {
                    tl.power = if (G.gp1.left_trigger > 0.9) 1.0 else 0.0
                    tr.power = if (G.gp1.right_trigger > 0.9) 1.0 else 0.0
                    bl.power = if (G.gp1.left_bumper) 1.0 else 0.0
                    br.power = if (G.gp1.right_bumper) 1.0 else 0.0
                }

                telemetry["Current State"] = "Button Power"
            }
        }

        suspendUntil { G.gp1.left_stick_button }
        joystickDrive
    }

    override suspend fun runSuspendOpMode() {
        suspendUntilStart()

        coroutineScope {
            launch {
                runStateMachine(buttonPower)
            }

            loopYieldWhile({ true }) {
                telemetry["odo right"] = G.chub.odoRight.currentPosition
                telemetry["odo aux"] = G.chub.odoAux.currentPosition
                telemetry["odo left"] = G.chub.odoLeft.currentPosition
            }
        }
    }
}