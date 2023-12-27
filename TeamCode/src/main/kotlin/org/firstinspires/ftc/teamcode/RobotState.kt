package org.firstinspires.ftc.teamcode

import arrow.optics.optics
import com.outoftheboxrobotics.suspendftc.Looper
import com.outoftheboxrobotics.tickt.TicketScheduler
import org.firstinspires.ftc.teamcode.actions.hardware.DriveState
import org.firstinspires.ftc.teamcode.actions.hardware.ExtensionState
import org.firstinspires.ftc.teamcode.actions.hardware.ImuState
import org.firstinspires.ftc.teamcode.actions.hardware.OuttakeState
import org.firstinspires.ftc.teamcode.hardware.ControlHubHardware
import org.firstinspires.ftc.teamcode.hardware.ExHubHardware
import org.firstinspires.ftc.teamcode.vision.VisionState

/**
 * An immutable class that holds the state of the robot, global variables, and hardware layer.
 *
 * Instances of this class are stored in a MutableStateFlow in the Globals object.
 *
 * @param mainLooper The main looper for the robot which interacts with the expansion hub.
 * @param driveLooper The looper for drive train actions which interacts with the control hub.
 * @param chub The control hub hardware layer.
 * @param ehub The expansion hub hardware layer.
 * @param ticketScheduler The ticket scheduler for the robot.
 * @param imuState IMU-related state.
 * @param driveState Drivetrain-related state.
 * @param extensionState Extension-related state.
 * @param outtakeState Outtake-related state.
 * @param visionState Vision-related state.
 */
@optics
data class RobotState(
    val mainLooper: Looper,
    val driveLooper: Looper = mainLooper,
    val chub: ControlHubHardware,
    val ehub: ExHubHardware,
    val ticketScheduler: TicketScheduler,


    val commandHandler: CommandHandler,

    val imuState: ImuState,
    val driveState: DriveState,
    val extensionState: ExtensionState,
    val outtakeState: OuttakeState,

    val visionState: VisionState
) { companion object }
