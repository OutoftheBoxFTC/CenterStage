package org.firstinspires.ftc.teamcode.opmodes.testing

import arrow.fx.coroutines.raceN
import com.outoftheboxrobotics.suspendftc.suspendFor
import com.outoftheboxrobotics.suspendftc.suspendUntil
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import kotlinx.coroutines.coroutineScope
import kotlinx.coroutines.launch
import org.firstinspires.ftc.teamcode.actions.hardware.ArmPosition
import org.firstinspires.ftc.teamcode.actions.hardware.ClawPosition
import org.firstinspires.ftc.teamcode.actions.hardware.IntakeTiltPosition
import org.firstinspires.ftc.teamcode.actions.hardware.LiftConfig
import org.firstinspires.ftc.teamcode.actions.hardware.TwistPosition
import org.firstinspires.ftc.teamcode.actions.hardware.closeClaws
import org.firstinspires.ftc.teamcode.actions.hardware.liftUpTo
import org.firstinspires.ftc.teamcode.actions.hardware.openClaws
import org.firstinspires.ftc.teamcode.actions.hardware.profileArm
import org.firstinspires.ftc.teamcode.actions.hardware.retractExtension
import org.firstinspires.ftc.teamcode.actions.hardware.retractLift
import org.firstinspires.ftc.teamcode.actions.hardware.setArmPosition
import org.firstinspires.ftc.teamcode.actions.hardware.setClawPos
import org.firstinspires.ftc.teamcode.actions.hardware.setTiltPosition
import org.firstinspires.ftc.teamcode.actions.hardware.setTwistPosition
import org.firstinspires.ftc.teamcode.opmodes.RobotOpMode
import org.firstinspires.ftc.teamcode.util.G
import org.firstinspires.ftc.teamcode.util.mainLoop
import org.firstinspires.ftc.teamcode.util.suspendUntilRisingEdge
import org.firstinspires.ftc.teamcode.util.use
import kotlin.coroutines.coroutineContext

@TeleOp
class TransferTest : RobotOpMode() {
    suspend fun pause() {
        gamepad1.rumble(200)

        raceN(
            coroutineContext,
            {
                suspendUntil { gamepad1.a }
            },
            {
                suspendUntilRisingEdge { gamepad1.x }
            }
        )

    }

    override suspend fun runSuspendOpMode() {
        closeClaws()
        setTwistPosition(TwistPosition.STRAIGHT)
        setTiltPosition(IntakeTiltPosition.LOW)
        setArmPosition(ArmPosition.NEUTRAL)

        coroutineScope {
            launch {
                mainLoop {
                    G.ehub.intakeRoller.power = if (gamepad1.left_bumper) -1.0 else 0.0
                }
            }.use {
                suspendUntilStart()
            }
        }

        while (true) {
            coroutineScope {
                retractLift()

                openClaws()
                suspendFor(100)

                pause()

                G.ehub.intakeRoller.power = -0.8

                profileArm(ArmPosition.TRANSFER)

                pause()

                coroutineScope {
                    launch {
                        retractExtension()
                        profileArm(ArmPosition.TRANSFER)
                    }
                    suspendFor(100)
                    setTiltPosition(IntakeTiltPosition.TRANSFER_FLAT)
                }

                pause()

                setTiltPosition(IntakeTiltPosition.TRANSFER.pos + 0.15)

                suspendFor(500)

                setTiltPosition(IntakeTiltPosition.TRANSFER)
                G.ehub.intakeRoller.power = 0.0

                pause()

                setClawPos(ClawPosition.BLACK_CLOSE)

                G.ehub.outtakeLift.power = -1.0
                G.ehub.extension.power = -1.0

                suspendFor(300)

                closeClaws()

                suspendFor(450)

                G.ehub.outtakeLift.power = 0.0
                G.ehub.extension.power = -0.5

                pause()

                setArmPosition(ArmPosition.TRANSFER.pos + 0.01)
                setTiltPosition(IntakeTiltPosition.POST_TRANSFER)

                G.ehub.intakeRoller.power = -0.8
                suspendFor(50)
                G.ehub.intakeRoller.power = 0.0

                suspendFor(100)

                pause()

                liftUpTo(LiftConfig.transferHeightMin)
                retractExtension()

                pause()

                setTiltPosition(IntakeTiltPosition.LOW)
            }

            suspendUntil { gamepad1.y }
        }
    }
}