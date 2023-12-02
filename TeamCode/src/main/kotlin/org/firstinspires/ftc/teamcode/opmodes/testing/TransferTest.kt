package org.firstinspires.ftc.teamcode.opmodes.testing

import com.outoftheboxrobotics.suspendftc.loopYieldWhile
import com.outoftheboxrobotics.suspendftc.suspendFor
import com.outoftheboxrobotics.suspendftc.suspendUntil
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.actions.hardware.ArmPosition
import org.firstinspires.ftc.teamcode.actions.hardware.IntakeTiltPosition
import org.firstinspires.ftc.teamcode.actions.hardware.TwistPosition
import org.firstinspires.ftc.teamcode.actions.hardware.setArmPosition
import org.firstinspires.ftc.teamcode.actions.hardware.setTiltPosition
import org.firstinspires.ftc.teamcode.actions.hardware.setTwistPosition
import org.firstinspires.ftc.teamcode.opmodes.RobotOpMode
import org.firstinspires.ftc.teamcode.util.G

@TeleOp
class TransferTest : RobotOpMode() {
    override suspend fun runSuspendOpMode() {
        suspendUntilStart()

        setTwistPosition(TwistPosition.STRAIGHT)

        while (true) {
            setArmPosition(ArmPosition.NEUTRAL)
            setTiltPosition(IntakeTiltPosition.LOW)
            suspendFor(500)

            loopYieldWhile({ !gamepad1.x }) {
                G.ehub.intakeRoller.power = when {
                    gamepad1.left_bumper -> -1.0
                    gamepad1.right_bumper -> 1.0
                    else -> 0.0
                }
            }

            G.ehub.intakeRoller.power = 0.0

            setArmPosition(ArmPosition.TRANSFER)
            suspendFor(200)
            setTiltPosition(IntakeTiltPosition.HIGH)
            suspendFor(600)
            setTiltPosition(IntakeTiltPosition.LOW)
            suspendFor(600)
            setArmPosition(ArmPosition.OUTTAKE)

            suspendUntil { gamepad1.y }
        }
    }
}