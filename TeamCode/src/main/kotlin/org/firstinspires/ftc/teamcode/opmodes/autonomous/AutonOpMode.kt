package org.firstinspires.ftc.teamcode.opmodes.autonomous

import org.firstinspires.ftc.teamcode.actions.hardware.ArmPosition
import org.firstinspires.ftc.teamcode.actions.hardware.IntakeTiltPosition
import org.firstinspires.ftc.teamcode.actions.hardware.TwistPosition
import org.firstinspires.ftc.teamcode.actions.hardware.closeClaws
import org.firstinspires.ftc.teamcode.actions.hardware.openClaws
import org.firstinspires.ftc.teamcode.actions.hardware.setArmPosition
import org.firstinspires.ftc.teamcode.actions.hardware.setTiltPosition
import org.firstinspires.ftc.teamcode.actions.hardware.setTwistPosition
import org.firstinspires.ftc.teamcode.opmodes.RobotOpMode
import org.firstinspires.ftc.teamcode.util.C
import org.firstinspires.ftc.teamcode.util.mainLoop

abstract class AutonOpMode : RobotOpMode() {
    suspend fun runAutonInit(): Nothing {
        setTwistPosition(TwistPosition.STRAIGHT)
        setArmPosition(ArmPosition.AUTON_INIT)
        setTiltPosition(IntakeTiltPosition.HIGH)

        mainLoop {
            if (C.openClaw) openClaws() else closeClaws()
        }
    }
}