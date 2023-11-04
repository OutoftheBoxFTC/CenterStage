package org.firstinspires.ftc.teamcode

import arrow.core.filterIsInstance
import arrow.core.none
import arrow.optics.Lens
import arrow.optics.Optional
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.hardware.ControlHubHardware
import org.firstinspires.ftc.teamcode.hardware.ExHubHardware
import org.firstinspires.ftc.teamcode.hardware.ThreadedImuHandler
import org.firstinspires.ftc.teamcode.opmodes.RobotOpMode

object Globals {
    private lateinit var robotState: RobotState
    private lateinit var currentOpMode: RobotOpMode

    fun defaultRobotState(hwMap: HardwareMap) = RobotState(
        looper = RobotLooper(),
        chub = ControlHubHardware(hwMap),
        ehub = ExHubHardware(hwMap),
        imuHandler = none(),
        commandHandler = null
    )

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

    operator fun <T> get(lens: Lens<RobotState, T>) = lens.get(robotState)
    operator fun <T> get(lens: Optional<RobotState, T>) = lens.getOrNull(robotState)
    operator fun <T> set(lens: Lens<RobotState, T>, value: T) {
        robotState = lens.set(robotState, value)
    }

    operator fun <T> set(lens: Optional<RobotState, T>, value: T) {
        robotState = lens.set(robotState, value)
    }
}
