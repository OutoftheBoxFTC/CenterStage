package org.firstinspires.ftc.teamcode.opmodes.testing

import arrow.core.nonEmptyListOf
import arrow.core.some
import com.outoftheboxrobotics.suspendftc.loopYieldWhile
import com.outoftheboxrobotics.suspendftc.suspendUntil
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import kotlinx.coroutines.coroutineScope
import kotlinx.coroutines.launch
import org.firstinspires.ftc.teamcode.C
import org.firstinspires.ftc.teamcode.FS
import org.firstinspires.ftc.teamcode.G
import org.firstinspires.ftc.teamcode.RobotState
import org.firstinspires.ftc.teamcode.actions.hardware.setDrivePowers
import org.firstinspires.ftc.teamcode.command.Subsystem
import org.firstinspires.ftc.teamcode.hardware.devices.ThreadedImuHandler
import org.firstinspires.ftc.teamcode.imuHandler
import org.firstinspires.ftc.teamcode.launchCommand
import org.firstinspires.ftc.teamcode.opmodes.RobotOpMode
import org.firstinspires.ftc.teamcode.set
import org.firstinspires.ftc.teamcode.statemachine.runStateMachine
import kotlin.math.cos
import kotlin.math.sin

@TeleOp
class DriveTest : RobotOpMode(
    runMultiThreaded = true,
    imuHandler = ThreadedImuHandler().some()
) {
    private inner class Fsm {
        val imu = G[RobotState.imuHandler].getOrNull()!!

        val joystickDrive: FS = FS {
            G.cmd.launchCommand(nonEmptyListOf(Subsystem.DRIVETRAIN)) {
                loopYieldWhile({ true }) {
                    if (C.imuResetAngle) imu.resetAngle()

                    val heading = imu.angle

                    setDrivePowers(
                        C.driveStrafeX * cos(-heading) - C.driveStrafeY * sin(-heading),
                        C.driveStrafeX * sin(-heading) + C.driveStrafeY * cos(-heading),
                        C.driveTurn
                    )
                    telemetry["Current State"] = "Joystick Drive"
                }
            }

            suspendUntil { G.gp1.a }
            buttonPower
        }

        val buttonPower: FS = FS {
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
    }

    override suspend fun runSuspendOpMode() {
        suspendUntilStart()

        coroutineScope {
            launch {
                runStateMachine(Fsm().buttonPower)
            }

            loopYieldWhile({ true }) {
                telemetry["odo right"] = G.chub.odoRight
                telemetry["odo aux"] = G.chub.odoAux
                telemetry["odo left"] = G.chub.odoLeft
            }
        }
    }
}