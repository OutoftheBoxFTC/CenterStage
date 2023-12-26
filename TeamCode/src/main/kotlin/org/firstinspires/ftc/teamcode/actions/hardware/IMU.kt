package org.firstinspires.ftc.teamcode.actions.hardware

import arrow.optics.optics
import com.acmerobotics.roadrunner.util.Angle
import com.qualcomm.robotcore.hardware.IMU
import kotlinx.coroutines.Job
import kotlinx.coroutines.coroutineScope
import kotlinx.coroutines.isActive
import kotlinx.coroutines.launch
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.Globals
import org.firstinspires.ftc.teamcode.RobotState
import org.firstinspires.ftc.teamcode.imuState
import org.firstinspires.ftc.teamcode.util.mainLoop
import org.firstinspires.ftc.teamcode.util.mapState
import org.firstinspires.ftc.teamcode.util.modify
import org.firstinspires.ftc.teamcode.util.next
import kotlin.coroutines.CoroutineContext

/**
 * @param rawAngle The raw angle of the IMU, in radians
 * @param angleBias The bias of the IMU, in radians
 * @param threadedImuJob The job that runs the threaded IMU handler, if applicable
 */
@optics
data class ImuState(
    val rawAngle: Double,
    val angleBias: Double,

    val threadedImuJob: Job? = null
) {
    val angle = Angle.normDelta(rawAngle - angleBias)

    companion object
}

/**
 * Runs the IMU on the drive looper.
 */
suspend fun runDefaultImuHandler(imu: IMU): Nothing = coroutineScope {
    imu.resetYaw()

    mainLoop {
        Globals[RobotState.imuState.rawAngle] = imu.robotYawPitchRollAngles.getYaw(AngleUnit.RADIANS)
    }
}

/**
 * Runs the IMU in its own thread, specified by [context].
 */
suspend fun runThreadedImuHandler(context: CoroutineContext, imu: IMU) = coroutineScope {
    imu.resetYaw()

    Globals[RobotState.imuState.threadedImuJob] = launch(context) {
        while (isActive) {
            Globals[RobotState.imuState.rawAngle] = imu.robotYawPitchRollAngles.getYaw(AngleUnit.RADIANS)
        }
    }
}

fun currentImuAngle() = Globals[RobotState.imuState].angle

fun resetImuAngle(newAngle: Double = 0.0) = Globals.robotState.modify {
    RobotState.imuState.angleBias set it.imuState.rawAngle - newAngle
}

/**
 * Suspends until the next IMU angle is available.
 */
suspend fun nextImuAngle() = Globals.robotState.mapState { it.imuState.angle }.next()
