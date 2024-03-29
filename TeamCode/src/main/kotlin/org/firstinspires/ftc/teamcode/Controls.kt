package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.Gamepad
import org.firstinspires.ftc.teamcode.util.ReadOnlyProperty
import kotlin.math.abs

/**
 * Delegated properties that map controls to gamepad inputs.
 *
 * [See Github issue for full explanation](https://github.com/OutoftheBoxFTC/CenterStage/issues/2)
 */
object Controls {
    // Autonomous
    val openTiltAuto by driverInput { right_bumper }
    val resetAutoPose by driverInput { left_bumper }
    val toggleAutoBreakpointEnable by driverInput { x }

    // Common
    val driveStrafeX by driverInput { -left_stick_y.toDouble() }
    val driveStrafeY by driverInput { -left_stick_x.toDouble() }
    val driveTurn by driverInput { -right_stick_x.toDouble() }

    val slowDrive by driverInput { left_bumper }
    val imuResetAngle by driverInput { x }

    val hangDroneEnable by operatorInput { left_trigger > 0.9 && right_trigger > 0.9 }

    val hang0 by operatorInput {
        if (hangDroneEnable) -right_stick_y.toDouble()
        else 0.0
    }

    val hang1 by operatorInput {
        if (hangDroneEnable) -left_stick_y.toDouble()
        else 0.0
    }

    val launchDrone by operatorInput { hangDroneEnable && left_stick_button && right_stick_button}


    val enterTransfer by operatorInput { x }
    val enterTransferAux by operatorInput { a }  // need this bc our operator is a dubass and keeps hitting the transfer button

    val toggleOperatorOverride by operatorInput { right_stick_button && !hangDroneEnable }

    val outtakeLow by operatorInput { dpad_down }
    val outtakeHigh by operatorInput { dpad_up }

    val autoAlign by driverInput { right_bumper }

    // Main/Intake State
    val driverExtend by driverInput { left_trigger > 0.9 }
    val driverRetract by driverInput { right_trigger > 0.9 }

    val startRoller by operatorInput { left_bumper }
    val reverseRoller by operatorInput { right_bumper }

    // Operator Override State
    val operatorExtension by operatorInput { if (hangDroneEnable) -0.15 else -left_stick_y.toDouble() }
    val operatorPivot by operatorInput { if (hangDroneEnable) 0.0 else -left_stick_x.toDouble() }

    val tiltToggleUp by operatorInput { y }
    val tiltToggleDown by operatorInput { a }

    // Transfer State
    val quitTransfer by operatorInput { b }

    // Outtake State
    val releaseClaw by driverInput { left_trigger > 0.9 || right_trigger > 0.9 }

    val operatorLiftUp by operatorInput { y }
    val operatorLiftDown by operatorInput { a }
    val exitOuttake by operatorInput { dpad_right }

    // Auto Align
    val driveAutoLift by driverInput { (-right_stick_x) }
    val driveHeadingAdjust by driverInput { (-right_stick_y).takeIf { abs(it) > 0.5 } ?: 0f }

    val driveTwistLeft by driverInput { left_trigger > 0.9 }
    val driveTwistRight by driverInput { right_trigger > 0.9 }

    // Property delegates
    private fun <T> driverInput(block: Gamepad.() -> T) = object : ReadOnlyProperty<T> {
        override val value get() = Globals.gp1.block()
    }

    private fun <T> operatorInput(block: Gamepad.() -> T) = object : ReadOnlyProperty<T> {
        override val value get() = Globals.gp2.block()
    }
}