package org.firstinspires.ftc.teamcode.hardware

import com.qualcomm.robotcore.hardware.HardwareMap

const val IMU_NAME = "imu"

class ControlHubHardware(hwMap: HardwareMap) : HardwareLayer(hwMap, "Control Hub") {
    val tr = motor("1")
    val tl = motor("2")
    val br = motor("0")
    val bl = motor("3")

    val odoRight by encoder("3")
    val odoAux by encoder("1")
    val odoLeft by encoder("0")
}

class ExHubHardware(hwMap: HardwareMap) : HardwareLayer(hwMap, "Expansion Hub 2") {

}
