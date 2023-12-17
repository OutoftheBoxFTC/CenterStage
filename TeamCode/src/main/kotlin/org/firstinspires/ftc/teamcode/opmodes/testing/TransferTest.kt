package org.firstinspires.ftc.teamcode.opmodes.testing

import com.outoftheboxrobotics.suspendftc.loopYieldWhile
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.actions.hardware.ArmPosition
import org.firstinspires.ftc.teamcode.actions.hardware.IntakeTiltPosition
import org.firstinspires.ftc.teamcode.actions.hardware.TwistPosition
import org.firstinspires.ftc.teamcode.actions.hardware.closeClaws
import org.firstinspires.ftc.teamcode.actions.hardware.openClaws
import org.firstinspires.ftc.teamcode.actions.hardware.setArmPosition
import org.firstinspires.ftc.teamcode.actions.hardware.setTiltPosition
import org.firstinspires.ftc.teamcode.actions.hardware.setTwistPosition
import org.firstinspires.ftc.teamcode.opmodes.RobotOpMode
import org.firstinspires.ftc.teamcode.util.G
import org.firstinspires.ftc.teamcode.util.suspendUntilRisingEdge

@TeleOp
@Disabled
class TransferTest : RobotOpMode() {
    override suspend fun runSuspendOpMode() {
        suspendUntilStart()

        openClaws()
        setTwistPosition(TwistPosition.STRAIGHT)

        while (true) {
            setArmPosition(ArmPosition.NEUTRAL)
            setTiltPosition(IntakeTiltPosition.LOW)
            setTwistPosition(TwistPosition.STRAIGHT)

            loopYieldWhile({ !gamepad1.x }) {
                G.ehub.intakeRoller.power = when {
                    gamepad1.left_bumper -> -1.0
                    gamepad1.right_bumper -> 1.0
                    else -> 0.0
                }
            }

            G.ehub.intakeRoller.power = -0.8

            setTiltPosition(IntakeTiltPosition.HIGH)
            pause()
            setArmPosition(ArmPosition.TRANSFER)
            pause()
            closeClaws()
            pause()
            setTiltPosition(IntakeTiltPosition.LOW)
            pause()
            setArmPosition(ArmPosition.OUTTAKE)
            pause()
            setTwistPosition(TwistPosition.HORIZONTAL)

            pause()

            setArmPosition(ArmPosition.FLOOR)
            pause()
            openClaws()
            pause()
        }
    }

    private suspend fun pause() = suspendUntilRisingEdge { gamepad1.b }
}