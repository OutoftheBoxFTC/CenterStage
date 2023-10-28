package org.firstinspires.ftc.teamcode

import arrow.optics.Lens
import arrow.optics.Optional
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.hardware.ControlHubHardware
import org.firstinspires.ftc.teamcode.hardware.ExHubHardware

object Globals {
    lateinit var robotState: RobotState
        private set

    fun defaultRobotState(hwMap: HardwareMap) = RobotState(
        looper = RobotLooper(),
        chub = ControlHubHardware(hwMap),
        ehub = ExHubHardware(hwMap)
    )

    operator fun <T> get(lens: Lens<RobotState, T>) = lens.get(robotState)
    operator fun <T> get(lens: Optional<RobotState, T>) = lens.getOrNull(robotState)
    operator fun <T> set(lens: Lens<RobotState, T>, value: T) = lens.set(robotState, value)
}
