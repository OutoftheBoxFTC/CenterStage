package org.firstinspires.ftc.teamcode

import arrow.optics.optics
import com.outoftheboxrobotics.suspendftc.Looper
import org.firstinspires.ftc.teamcode.actions.hardware.DriveState
import org.firstinspires.ftc.teamcode.actions.hardware.ExtensionState
import org.firstinspires.ftc.teamcode.actions.hardware.ImuState
import org.firstinspires.ftc.teamcode.hardware.ControlHubHardware
import org.firstinspires.ftc.teamcode.hardware.ExHubHardware
import org.firstinspires.ftc.teamcode.vision.VisionState

@optics
data class RobotState(
    val mainLooper: Looper,
    val driveLooper: Looper = mainLooper,
    val chub: ControlHubHardware,
    val ehub: ExHubHardware,
    val commandHandler: CommandHandler,

    val imuState: ImuState,
    val driveState: DriveState,
    val extensionState: ExtensionState,

    val visionState: VisionState
) { companion object }
