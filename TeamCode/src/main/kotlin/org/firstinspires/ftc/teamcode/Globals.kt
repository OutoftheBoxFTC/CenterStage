package org.firstinspires.ftc.teamcode

import arrow.core.filterIsInstance
import arrow.core.none
import arrow.optics.Lens
import arrow.optics.Optional
import com.outoftheboxrobotics.suspendftc.Looper
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.HardwareMap
import kotlinx.coroutines.flow.MutableStateFlow
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.command.CommandHandler
import org.firstinspires.ftc.teamcode.subsystems.RoadrunnerDrivetrain
import org.firstinspires.ftc.teamcode.hardware.ControlHubHardware
import org.firstinspires.ftc.teamcode.hardware.ExHubHardware
import org.firstinspires.ftc.teamcode.hardware.devices.ThreadedImuHandler
import org.firstinspires.ftc.teamcode.logging.Loggers
import org.firstinspires.ftc.teamcode.opmodes.RobotOpMode
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive

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
            drivetrainHandler = RoadrunnerDrivetrain { SampleMecanumDrive(chubLayer) },
            imuHandler = none(),
            commandHandler = CommandHandler.new(),
            loggers = Loggers(telemetry)
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

        state.imuHandler.filterIsInstance<ThreadedImuHandler>().getOrNull()?.cancel()
    }

    val chub get() = robotState.value.chub
    val ehub get() = robotState.value.ehub

    val gp1: Gamepad get() = currentOpMode.gamepad1
    val gp2: Gamepad get() = currentOpMode.gamepad2

    val cmd get() = robotState.value.commandHandler

    val drive get() = robotState.value.drivetrainHandler

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
