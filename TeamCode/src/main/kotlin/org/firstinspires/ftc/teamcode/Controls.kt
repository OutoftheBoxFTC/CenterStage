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
    val openClaw by driverInput { right_bumper }
    val toggleAutoBreakpointEnable by driverInput { x }

    // Common
    val driveStrafeX by driverInput { -left_stick_y.toDouble() }
    val driveStrafeY by driverInput { -left_stick_x.toDouble() }
    val driveTurn by driverInput { -right_stick_x.toDouble() }

    val slowDrive by driverInput { left_bumper }
    val imuResetAngle by driverInput { x }

    val hang0 by driverInput { when {
        dpad_up -> -1.0
        dpad_down -> 1.0
        else -> 0.0
    } }

    val hang1 by driverInput { when {
        y -> -1.0
        a -> 1.0
        else -> 0.0
    } }

    val launchDrone by operatorInput { left_stick_button && right_stick_button && left_trigger > 0.9 && right_trigger > 0.9}


    val enterTransfer by operatorInput { x }
    val enterTransferAux by operatorInput { a }  // need this bc our operator is a dubass and keeps hitting the transfer button

    val toggleOperatorOverride by operatorInput { right_stick_button && left_trigger < 0.9 && right_trigger < 0.9 }

    val outtakeLow by operatorInput { dpad_down }
    val outtakeHigh by operatorInput { dpad_up }

    val autoAlign by driverInput { right_bumper }

    // Main/Intake State
    val driverExtend by driverInput { left_trigger > 0.9 }
    val driverRetract by driverInput { right_trigger > 0.9 }

    val startRoller by operatorInput { left_bumper }
    val reverseRoller by operatorInput { right_bumper }

    // Operator Override State
    val operatorExtension by operatorInput { -left_stick_y }
    val operatorPivot by operatorInput { -left_stick_x }

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
    val driveAutoLift by driverInput { -right_stick_y }
    val driveHeadingAdjust by driverInput { (-right_stick_x).takeIf { abs(it) > 0.5 } ?: 0f }

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