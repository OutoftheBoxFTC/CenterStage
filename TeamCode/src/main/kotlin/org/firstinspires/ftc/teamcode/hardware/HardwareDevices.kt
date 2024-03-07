package org.firstinspires.ftc.teamcode.hardware

import arrow.core.nonEmptyListOf
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.DigitalChannel
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.VoltageSensor
import org.firstinspires.ftc.teamcode.hardware.devices.MotorGroup
import org.firstinspires.ftc.teamcode.hardware.devices.ServoGroup
import org.firstinspires.ftc.teamcode.roadrunner.util.Encoder

const val IMU_NAME = "imu"

// Referenced in RoadRunner SampleMecanumDrive
object DriveHardwareNames {
    const val tr = "1"
    const val tl = "2"
    const val br = "0"
    const val bl = "3"

    const val odoRight = "3"
    const val odoAux = "1"
    const val odoLeft = "0"
}

/**
 * Hardware layer for the control hub.
 */
class ControlHubHardware(hwMap: HardwareMap) : HardwareLayer(hwMap, "Control Hub") {
    val voltageSensor: VoltageSensor = hwMap.voltageSensor.first()

    // Drive motors
    // Top is intake side
    val tr = motor(DriveHardwareNames.tr)
    val tl = motor(DriveHardwareNames.tl)
    val br = motor(DriveHardwareNames.br)
    val bl = motor(DriveHardwareNames.bl)

    val odoRight = motor(DriveHardwareNames.odoRight)
    val odoAux = motor(DriveHardwareNames.odoAux)
    val odoLeft = motor(DriveHardwareNames.odoLeft)

    val intakeTilt = servo("s0")

    val hang0 = crServo("s1")

    private val odoIntakeMotor = motor("2")
    val odoIntake = Encoder(odoIntakeMotor)

    val arm = ServoGroup(
        nonEmptyListOf(
            // Right
            servo("s3"),

            // Left
            servo("s2")
        )
    )

    val blackClaw = servo("s4")
    val redClaw = servo("s5")

    private val extensionLimitSwitchDevice =
        hwMap[DigitalChannel::class.java, "extensionLimitSwitch"]

    val extensionLimitSwitch by inputField(true) {
        !extensionLimitSwitchDevice.state
    }

    val outtakeCamera = webcam("Outtake Webcam", true)
    val intakeCamera = webcam("Intake Webcam", false)
}

/**
 * Hardware layer for the expansion hub.
 */
class ExHubHardware(hwMap: HardwareMap) : HardwareLayer(hwMap, "Expansion Hub 2") {
    val extension = MotorGroup(
        nonEmptyListOf(
            // Left
            motor("4") { direction = DcMotorSimple.Direction.REVERSE },

            // Right
            motor("5")
        )
    )

    val twist = servo("s11")

    val intakeRoller = motor("7") {
        direction = DcMotorSimple.Direction.REVERSE
        zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT
    }

    val intakeWheel = crServo("s6") {
        direction = DcMotorSimple.Direction.REVERSE
    }

    val outtakeLift = motor("6")

    val hang1 = crServo("s8")

    val drone = servo("s10")
}
