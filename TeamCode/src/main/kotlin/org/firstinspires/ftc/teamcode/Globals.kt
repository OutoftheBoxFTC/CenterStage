package org.firstinspires.ftc.teamcode

import arrow.optics.Lens
import arrow.optics.Optional
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.command.CommandHandler
import org.firstinspires.ftc.teamcode.hardware.ControlHubHardware
import org.firstinspires.ftc.teamcode.hardware.DefaultImuHandler
import org.firstinspires.ftc.teamcode.hardware.ExHubHardware
import org.firstinspires.ftc.teamcode.hardware.ThreadedImuHandler

object Globals {
    private lateinit var robotState: RobotState

    fun defaultRobotState(hwMap: HardwareMap) = RobotState(
        looper = RobotLooper(),
        chub = ControlHubHardware(hwMap),
        ehub = ExHubHardware(hwMap),
        imuHandler = DefaultImuHandler(),
        commandHandler = CommandHandler()
    )

    fun initializeRobotState(state: RobotState) {
        robotState = state
    }

    fun stop() {
        robotState.looper.mainLooper.cancel()
        robotState.looper.driveLooper?.cancel()

        (robotState.imuHandler as? ThreadedImuHandler)?.cancel()
    }

    val chub get() = robotState.chub
    val ehub get() = robotState.ehub

    operator fun <T> get(lens: Lens<RobotState, T>) = lens.get(robotState)
    operator fun <T> get(lens: Optional<RobotState, T>) = lens.getOrNull(robotState)
    operator fun <T> set(lens: Lens<RobotState, T>, value: T) {
        robotState = lens.set(robotState, value)
    }

    operator fun <T> set(lens: Optional<RobotState, T>, value: T) {
        robotState = lens.set(robotState, value)
    }
}
