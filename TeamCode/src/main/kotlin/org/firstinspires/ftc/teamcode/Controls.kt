package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.Gamepad
import org.firstinspires.ftc.teamcode.util.ReadOnlyProperty

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

    // Default Main/Intake State
    val extendExtension by driverInput { left_trigger > 0.9 }
    val retractExtension by driverInput { right_trigger > 0.9 }

    val tiltToggle by driverInput { b }

    val runTransfer by operatorInput { x }
    val stopTransfer by operatorInput { b }

    val intakeRoller by operatorInput { left_bumper }
    val expelRoller by operatorInput { right_bumper }

    val outtakeLow by operatorInput { dpad_down }
    val outtakeHigh by operatorInput { dpad_up }

    // Outtake State
    val releaseLeftClaw by driverInput { left_trigger > 0.9 || right_trigger > 0.9 }
    val releaseRightClaw by driverInput { right_trigger > 0.9 || left_trigger > 0.9 }

    val autoPosition by driverInput { right_bumper }

    val twistClawLeft by operatorInput { x }
    val twistClawRight by operatorInput { b }

    val liftUp by operatorInput { y }
    val liftDown by operatorInput { a }

    val liftSlow by operatorInput { !left_bumper }

    val exitOuttake get() = Globals.gp1.b || Globals.gp2.dpad_right

    // Property delegates
    private fun <T> driverInput(block: Gamepad.() -> T) = object : ReadOnlyProperty<T> {
        override val value get() = Globals.gp1.block()
    }

    private fun <T> operatorInput(block: Gamepad.() -> T) = object : ReadOnlyProperty<T> {
        override val value get() = Globals.gp2.block()
    }
}