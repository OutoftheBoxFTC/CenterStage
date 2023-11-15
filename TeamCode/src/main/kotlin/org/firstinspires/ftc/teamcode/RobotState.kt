package org.firstinspires.ftc.teamcode

import arrow.core.Option
import arrow.optics.optics
import com.outoftheboxrobotics.suspendftc.Looper
import org.firstinspires.ftc.teamcode.command.CommandHandler
import org.firstinspires.ftc.teamcode.hardware.ControlHubHardware
import org.firstinspires.ftc.teamcode.hardware.ExHubHardware
import org.firstinspires.ftc.teamcode.hardware.devices.IMUHandler
import org.firstinspires.ftc.teamcode.logging.Loggers

@optics
data class RobotState(
    val looper: RobotLooper,
    val chub: ControlHubHardware,
    val ehub: ExHubHardware,
    val imuHandler: Option<IMUHandler>,
    val commandHandler: CommandHandler,
    val loggers: Loggers
) { companion object }

@optics
data class RobotLooper(
    val mainLooper: Looper = Looper(),
    val driveLooper: Looper? = null
) { companion object }
