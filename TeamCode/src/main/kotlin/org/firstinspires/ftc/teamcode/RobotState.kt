package org.firstinspires.ftc.teamcode

import arrow.optics.optics
import com.outoftheboxrobotics.suspendftc.Looper
import org.firstinspires.ftc.teamcode.hardware.ControlHubHardware
import org.firstinspires.ftc.teamcode.hardware.ExHubHardware

@optics
data class RobotState(
    val looper: RobotLooper,
    val chub: ControlHubHardware,
    val ehub: ExHubHardware
) { companion object }

@optics
data class RobotLooper(
    val mainLooper: Looper = Looper(),
    val driveLooper: Looper? = null
) { companion object }
