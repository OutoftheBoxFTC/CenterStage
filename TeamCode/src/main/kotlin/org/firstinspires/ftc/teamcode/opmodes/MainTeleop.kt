package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.actions.hardware.ArmPosition
import org.firstinspires.ftc.teamcode.actions.hardware.IntakeTiltPosition
import org.firstinspires.ftc.teamcode.actions.hardware.TwistPosition
import org.firstinspires.ftc.teamcode.actions.hardware.openClaws
import org.firstinspires.ftc.teamcode.actions.hardware.retractExtension
import org.firstinspires.ftc.teamcode.actions.hardware.setArmPosition
import org.firstinspires.ftc.teamcode.actions.hardware.setTiltPosition
import org.firstinspires.ftc.teamcode.actions.hardware.setTwistPosition
import org.firstinspires.ftc.teamcode.runStateMachine
import org.firstinspires.ftc.teamcode.util.FS

/**
 * Main Teleop program.
 */
@TeleOp
class MainTeleop : RobotOpMode() {
    private val mainState: FS = FS {
        TODO()
    }

    private val outtakeState: FS = FS {
        TODO()
    }

    override suspend fun runSuspendOpMode() {
        setArmPosition(ArmPosition.NEUTRAL)
        setTwistPosition(TwistPosition.STRAIGHT)
        setTiltPosition(IntakeTiltPosition.HIGH)
        openClaws()

        retractExtension()

        suspendUntilStart()

        runStateMachine(mainState)
    }
}