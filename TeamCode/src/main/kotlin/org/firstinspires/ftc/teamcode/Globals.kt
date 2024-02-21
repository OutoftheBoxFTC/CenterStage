package org.firstinspires.ftc.teamcode

import arrow.optics.Lens
import arrow.optics.Optional
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.outoftheboxrobotics.suspendftc.Looper
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.HardwareMap
import kotlinx.coroutines.flow.MutableStateFlow
import org.firstinspires.ftc.teamcode.actions.hardware.DriveControlState
import org.firstinspires.ftc.teamcode.actions.hardware.DriveState
import org.firstinspires.ftc.teamcode.actions.hardware.ExtensionState
import org.firstinspires.ftc.teamcode.actions.hardware.ImuState
import org.firstinspires.ftc.teamcode.actions.hardware.OuttakeState
import org.firstinspires.ftc.teamcode.hardware.ControlHubHardware
import org.firstinspires.ftc.teamcode.hardware.ExHubHardware
import org.firstinspires.ftc.teamcode.opmodes.RobotOpMode
import org.firstinspires.ftc.teamcode.vision.VisionState

/**
 * Global static object that holds the robot state and other global variables.
 */
object Globals {
    var imuStartingHeading: Double? = null

    /**
     * Mutable state flow that holds the robot state.
     */
    lateinit var robotState: MutableStateFlow<RobotState>
        private set

    /**
     * The current OpMode.
     */
    private lateinit var currentOpMode: RobotOpMode

    /**
     * Creates a default robot state.
     *
     * @param hwMap The hardware map to use.
     */
    fun defaultRobotState(hwMap: HardwareMap): RobotState {
        val chubLayer = ControlHubHardware(hwMap)
        val mainLooper = Looper()

        return RobotState(
            mainLooper = mainLooper,
            driveLooper = mainLooper,
            chub = chubLayer,
            ehub = ExHubHardware(hwMap),
            commandHandler = CommandHandler.new(),

            imuState = ImuState(0.0, 0.0),
            driveState = DriveState(null, Pose2d(), Pose2d(), DriveControlState.Idle),
            extensionState = ExtensionState(0),
            outtakeState = OuttakeState(),
            visionState = VisionState()
        )
    }

    /**
     * Resets variables to use the given robot state and OpMode.
     */
    fun initializeRobotState(state: RobotState, opMode: RobotOpMode) {
        robotState = MutableStateFlow(state)
        currentOpMode = opMode
    }

    /**
     * Stops coroutines and other resources.
     */
    fun stop() {
        val state = robotState.value

        state.mainLooper.cancel()

        // Make sure we don't call cancel() twice
        state.driveLooper.takeUnless { it == state.mainLooper }?.cancel()

        state.imuState.threadedImuJob?.cancel()
    }

    // Check if opmode in init
    val isInit get() = currentOpMode.opModeInInit()

    // Convenience accessors for the control and expansion hub hardware layers.
    val chub get() = robotState.value.chub
    val ehub get() = robotState.value.ehub

    // Convenience accessors for the gamepads.
    val gp1: Gamepad get() = currentOpMode.gamepad1
    val gp2: Gamepad get() = currentOpMode.gamepad2


    // Convenience accessors for the command handler.
    val cmd get() = robotState.value.commandHandler

    // Gets members of the robot state using arrow-optics lenses.
    operator fun <T> get(lens: Lens<RobotState, T>) = lens.get(robotState.value)
    operator fun <T> get(lens: Optional<RobotState, T>) = lens.getOrNull(robotState.value)

    // Sets members of the robot state using arrow-optics lenses.
    operator fun <T> set(lens: Lens<RobotState, T>, value: T) {
        robotState.value = lens.set(robotState.value, value)
    }

    operator fun <T> set(lens: Optional<RobotState, T>, value: T) {
        robotState.value = lens.set(robotState.value, value)
    }
}
