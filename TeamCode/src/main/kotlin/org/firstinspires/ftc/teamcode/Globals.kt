package org.firstinspires.ftc.teamcode

import arrow.optics.Lens
import arrow.optics.Optional
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.profile.MotionState
import com.outoftheboxrobotics.suspendftc.Looper
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.HardwareMap
import kotlinx.coroutines.flow.MutableStateFlow
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.actions.hardware.DriveControlState
import org.firstinspires.ftc.teamcode.actions.hardware.DriveState
import org.firstinspires.ftc.teamcode.actions.hardware.ExtensionState
import org.firstinspires.ftc.teamcode.actions.hardware.ImuState
import org.firstinspires.ftc.teamcode.hardware.ControlHubHardware
import org.firstinspires.ftc.teamcode.hardware.ExHubHardware
import org.firstinspires.ftc.teamcode.logging.Loggers
import org.firstinspires.ftc.teamcode.opmodes.RobotOpMode

object Globals {
    lateinit var robotState: MutableStateFlow<RobotState>
        private set

    private lateinit var currentOpMode: RobotOpMode

    fun defaultRobotState(hwMap: HardwareMap, telemetry: Telemetry): RobotState {
        val chubLayer = ControlHubHardware(hwMap)
        val mainLooper = Looper()

        return RobotState(
            mainLooper = mainLooper,
            driveLooper = mainLooper,
            chub = chubLayer,
            ehub = ExHubHardware(hwMap),
            commandHandler = CommandHandler.new(),
            loggers = Loggers(telemetry),

            imuState = ImuState(0.0, 0.0),
            driveState = DriveState(Pose2d(), DriveControlState.Idle),
            extensionState = ExtensionState(MotionState(0.0, 0.0))
        )
    }

    fun initializeRobotState(state: RobotState, opMode: RobotOpMode) {
        robotState = MutableStateFlow(state)
        currentOpMode = opMode
    }

    fun stop() {
        val state = robotState.value

        state.mainLooper.cancel()
        state.driveLooper.takeUnless { it == state.mainLooper }?.cancel()

        state.imuState.threadedImuJob?.cancel()
    }

    val chub get() = robotState.value.chub
    val ehub get() = robotState.value.ehub

    val gp1: Gamepad get() = currentOpMode.gamepad1
    val gp2: Gamepad get() = currentOpMode.gamepad2

    val cmd get() = robotState.value.commandHandler

    val log get() = robotState.value.loggers

    operator fun <T> get(lens: Lens<RobotState, T>) = lens.get(robotState.value)
    operator fun <T> get(lens: Optional<RobotState, T>) = lens.getOrNull(robotState.value)
    operator fun <T> set(lens: Lens<RobotState, T>, value: T) {
        robotState.value = lens.set(robotState.value, value)
    }

    operator fun <T> set(lens: Optional<RobotState, T>, value: T) {
        robotState.value = lens.set(robotState.value, value)
    }
}
