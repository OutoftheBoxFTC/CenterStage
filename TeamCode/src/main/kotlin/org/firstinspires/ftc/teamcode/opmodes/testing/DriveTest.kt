package org.firstinspires.ftc.teamcode.opmodes.testing

import arrow.fx.coroutines.raceN
import com.outoftheboxrobotics.suspendftc.loopYieldWhile
import com.outoftheboxrobotics.suspendftc.suspendUntil
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import kotlinx.coroutines.coroutineScope
import kotlinx.coroutines.launch
import org.firstinspires.ftc.teamcode.Subsystem
import org.firstinspires.ftc.teamcode.actions.hardware.runFieldCentricDrive
import org.firstinspires.ftc.teamcode.actions.hardware.setDrivePowers
import org.firstinspires.ftc.teamcode.opmodes.RobotOpMode
import org.firstinspires.ftc.teamcode.runStateMachine
import org.firstinspires.ftc.teamcode.util.C
import org.firstinspires.ftc.teamcode.util.FS
import org.firstinspires.ftc.teamcode.util.G
import org.firstinspires.ftc.teamcode.util.launchTicket
import org.firstinspires.ftc.teamcode.util.mainLoop
import org.firstinspires.ftc.teamcode.util.set

@TeleOp
@Disabled
class DriveTest : RobotOpMode() {
    private val joystickDrive: FS by lazy { FS {
        launchTicket(Subsystem.DRIVETRAIN) {
            launch {
                mainLoop {
                    telemetry["Current State"] = "Joystick Drive"
                }
            }

            runFieldCentricDrive()
        }

        joystickDrive.getNextState()
    } }

    private val intakeJoystickDrive: FS by lazy { FS {
        launchTicket(Subsystem.DRIVETRAIN) {
            mainLoop {
                setDrivePowers(
                    0.0,
                    0.0,
                    C.driveStrafeY
                )

                G.ehub.extension.power = C.driveStrafeX
                G.chub.intakeWheel.power = C.driveStrafeY

                telemetry["Current State"] = "Intake"
            }
        }

        intakeJoystickDrive.getNextState()
    } }

    private val buttonPower: FS by lazy { FS {
        launchTicket(Subsystem.DRIVETRAIN) {
            mainLoop {
                with(G.chub) {
                    tl.power = if (G.gp1.left_trigger > 0.9) 1.0 else 0.0
                    tr.power = if (G.gp1.right_trigger > 0.9) 1.0 else 0.0
                    bl.power = if (G.gp1.left_bumper) 1.0 else 0.0
                    br.power = if (G.gp1.right_bumper) 1.0 else 0.0
                }

                telemetry["Current State"] = "Button Power"
            }
        }

        buttonPower.getNextState()
    } }

    private suspend fun FS.getNextState(): FS = coroutineScope {
        return@coroutineScope raceN(
            coroutineContext,
            {
                suspendUntil { G.gp1.left_stick_button }
                buttonPower.also { suspendUntil { it != this@getNextState } }
            },
            {
                suspendUntil { G.gp1.a }
                joystickDrive.also { suspendUntil { it != this@getNextState } }
            },
            {
                suspendUntil { G.gp1.x }
                intakeJoystickDrive.also { suspendUntil { it != this@getNextState } }
            }
        ).fold({ it }, { it }, { it })
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