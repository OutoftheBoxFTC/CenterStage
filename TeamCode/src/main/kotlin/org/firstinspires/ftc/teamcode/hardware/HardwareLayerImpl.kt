package org.firstinspires.ftc.teamcode.hardware

import arrow.core.nonEmptyListOf
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.VoltageSensor
import org.firstinspires.ftc.teamcode.hardware.devices.MotorGroup

const val IMU_NAME = "imu"

object DriveHardwareNames {
    const val tr = "1"
    const val tl = "2"
    const val br = "0"
    const val bl = "3"

    const val odoRight = "3"
    const val odoAux = "1"
    const val odoLeft = "0"
}

class ControlHubHardware(hwMap: HardwareMap) : HardwareLayer(hwMap, "Control Hub") {
    val voltageSensor: VoltageSensor = hwMap.voltageSensor.first()

    val tr = motor(DriveHardwareNames.tr)
    val tl = motor(DriveHardwareNames.tl)
    val br = motor(DriveHardwareNames.br)
    val bl = motor(DriveHardwareNames.bl)

    val odoRight = motor(DriveHardwareNames.odoRight)
    val odoAux = motor(DriveHardwareNames.odoAux)
    val odoLeft = motor(DriveHardwareNames.odoLeft)
}

class ExHubHardware(hwMap: HardwareMap) : HardwareLayer(hwMap, "Expansion Hub 2") {
    val extension = MotorGroup(
        nonEmptyListOf(
            // Left
            motor("4"),

            // Right
            motor("5") { direction = DcMotorSimple.Direction.REVERSE }
        )
    )

    val intakeRoller = motor("7")

    val intakeTilt = servo("s6")

    val outtakeLift = motor("6")
}
