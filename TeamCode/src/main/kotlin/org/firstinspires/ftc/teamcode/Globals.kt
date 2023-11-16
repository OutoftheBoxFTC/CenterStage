package org.firstinspires.ftc.teamcode

import arrow.core.filterIsInstance
import arrow.core.none
import arrow.optics.Lens
import arrow.optics.Optional
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.command.CommandHandler
import org.firstinspires.ftc.teamcode.drive.RoadrunnerDrivetrain
import org.firstinspires.ftc.teamcode.hardware.ControlHubHardware
import org.firstinspires.ftc.teamcode.hardware.ExHubHardware
import org.firstinspires.ftc.teamcode.hardware.devices.ThreadedImuHandler
import org.firstinspires.ftc.teamcode.logging.Loggers
import org.firstinspires.ftc.teamcode.opmodes.RobotOpMode
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive

object Globals {
    private lateinit var robotState: RobotState
    private lateinit var currentOpMode: RobotOpMode

    fun defaultRobotState(hwMap: HardwareMap, telemetry: Telemetry): RobotState {
        val chubLayer = ControlHubHardware(hwMap)

        return RobotState(
            looper = RobotLooper(),
            chub = chubLayer,
            ehub = ExHubHardware(hwMap),
            drivetrainHandler = RoadrunnerDrivetrain(SampleMecanumDrive(chubLayer)),
            imuHandler = none(),
            commandHandler = CommandHandler.new(),
            loggers = Loggers(telemetry)
        )
    }

    fun initializeRobotState(state: RobotState, opMode: RobotOpMode) {
        robotState = state
        currentOpMode = opMode
    }

    fun stop() {
        robotState.looper.mainLooper.cancel()
        robotState.looper.driveLooper?.cancel()

        robotState.imuHandler.filterIsInstance<ThreadedImuHandler>().getOrNull()?.cancel()
    }

    val chub get() = robotState.chub
    val ehub get() = robotState.ehub

    val gp1 get() = currentOpMode.gamepad1
    val gp2 get() = currentOpMode.gamepad2

    val cmd get() = robotState.commandHandler

    val drive get() = robotState.drivetrainHandler

    val log get() = robotState.loggers

    operator fun <T> get(lens: Lens<RobotState, T>) = lens.get(robotState)
    operator fun <T> get(lens: Optional<RobotState, T>) = lens.getOrNull(robotState)
    operator fun <T> set(lens: Lens<RobotState, T>, value: T) {
        robotState = lens.set(robotState, value)
    }

    operator fun <T> set(lens: Optional<RobotState, T>, value: T) {
        robotState = lens.set(robotState, value)
    }
}
