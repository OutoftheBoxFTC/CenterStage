package org.firstinspires.ftc.teamcode.actions.hardware

import arrow.optics.optics
import com.acmerobotics.roadrunner.util.Angle
import com.outoftheboxrobotics.suspendftc.loopYieldWhile
import com.qualcomm.robotcore.hardware.IMU
import kotlinx.coroutines.Job
import kotlinx.coroutines.coroutineScope
import kotlinx.coroutines.isActive
import kotlinx.coroutines.launch
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.Globals
import org.firstinspires.ftc.teamcode.RobotState
import org.firstinspires.ftc.teamcode.imuState
import org.firstinspires.ftc.teamcode.util.mapState
import org.firstinspires.ftc.teamcode.util.modify
import org.firstinspires.ftc.teamcode.util.next
import kotlin.coroutines.CoroutineContext

@optics
data class ImuState(
    val rawAngle: Double,
    val angleBias: Double,

    val threadedImuJob: Job? = null
) {
    val angle = Angle.normDelta(rawAngle - angleBias)

    companion object
}

suspend fun runDefaultImuHandler(imu: IMU): Nothing = coroutineScope {
    imu.resetYaw()

    loopYieldWhile({ true }) {
        Globals[RobotState.imuState.rawAngle] = imu.robotYawPitchRollAngles.getYaw(AngleUnit.RADIANS)

        val imuState = Globals[RobotState.imuState]

        Globals.log.imu["raw_angle"] = imuState.rawAngle
        Globals.log.imu["corrected_angle"] = imuState.angle

        Globals.log.imu.collect()
    }

    error("Return from DefaultImuHandler.runHandler()")
}

suspend fun runThreadedImuHandler(context: CoroutineContext, imu: IMU) = coroutineScope {
    imu.resetYaw()

    Globals[RobotState.imuState.threadedImuJob] = launch(context) {
        while (isActive) {
            Globals[RobotState.imuState.rawAngle] = imu.robotYawPitchRollAngles.getYaw(AngleUnit.RADIANS)

            val imuState = Globals[RobotState.imuState]

            Globals.log.imu["raw_angle"] = imuState.rawAngle
            Globals.log.imu["corrected_angle"] = imuState.angle

            Globals.log.imu.collect()
        }
    }
}

fun currentImuAngle() = Globals[RobotState.imuState].angle

fun resetImuAngle(newAngle: Double = 0.0) = Globals.robotState.modify {
    RobotState.imuState.angleBias set it.imuState.rawAngle - newAngle
}
suspend fun nextImuAngle() = Globals.robotState.mapState { it.imuState.angle }.next()
