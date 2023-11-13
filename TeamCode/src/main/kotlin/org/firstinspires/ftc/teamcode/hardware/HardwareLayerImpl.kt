package org.firstinspires.ftc.teamcode.hardware

import com.qualcomm.robotcore.hardware.HardwareMap

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
    var enableDriveMotors = true

    val tr = motor(DriveHardwareNames.tr).apply { enableWrites = ::enableDriveMotors }
    val tl = motor(DriveHardwareNames.tl).apply { enableWrites = ::enableDriveMotors }
    val br = motor(DriveHardwareNames.br).apply { enableWrites = ::enableDriveMotors }
    val bl = motor(DriveHardwareNames.bl).apply { enableWrites = ::enableDriveMotors }

    val odoRight by encoder(DriveHardwareNames.odoRight)
    val odoAux by encoder(DriveHardwareNames.odoAux)
    val odoLeft by encoder(DriveHardwareNames.odoLeft)
}

class ExHubHardware(hwMap: HardwareMap) : HardwareLayer(hwMap, "Expansion Hub 2") {

}
