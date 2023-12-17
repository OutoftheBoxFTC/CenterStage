package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.Gamepad
import org.firstinspires.ftc.teamcode.util.ReadOnlyProperty

object Controls {
    // Autonomous
    val openClaw by driverInput { right_bumper }

    // Common
    val driveStrafeX by driverInput { -left_stick_y.toDouble() }
    val driveStrafeY by driverInput { -left_stick_x.toDouble() }
    val driveTurn by driverInput { -right_stick_x.toDouble() }

    val slowDrive by driverInput { left_bumper || lockHeading }
    val imuResetAngle by driverInput { right_bumper }

    // Main state
    val enterMainState by operatorInput { dpad_right }

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

    // Intake state
    val enterIntakeState by operatorInput { dpad_left }
    val exitIntakeNoTransfer by operatorInput { x }

    val intakeTiltUp by operatorInput { y }
    val intakeTiltDown by operatorInput { a }

    val rollerIn by operatorInput { left_bumper }
    val rollerOut by operatorInput { right_bumper }

    val operatorIntakeExtension by operatorInput { -left_stick_y.toDouble() }
    val operatorTurn by operatorInput { -left_stick_x.toDouble() }

    // Outtake state
    val enterOuttakeState by operatorInput { b }

    val lockHeading by driverInput { left_trigger > 0.9 }

    val liftUp by operatorInput { y }
    val liftDown by operatorInput { a }

    val clawRotateLeft by operatorInput { x }
    val clawRotateRight by operatorInput { b }

    val releaseFarPixel by operatorInput { left_bumper }
    val releaseClosePixel by operatorInput { right_bumper }


    private fun <T> driverInput(block: Gamepad.() -> T) = object : ReadOnlyProperty<T> {
        override val value get() = Globals.gp1.block()
    }

    private fun <T> operatorInput(block: Gamepad.() -> T) = object : ReadOnlyProperty<T> {
        override val value get() = Globals.gp2.block()
    }
}