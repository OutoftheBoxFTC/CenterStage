package org.firstinspires.ftc.teamcode.hardware

import com.qualcomm.robotcore.hardware.HardwareMap

const val IMU_NAME = "imu"

class ControlHubHardware(hwMap: HardwareMap) : HardwareLayer(hwMap, "Control Hub") {

}

class ExHubHardware(hwMap: HardwareMap) : HardwareLayer(hwMap, "Expansion Hub") {

}
