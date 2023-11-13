package org.firstinspires.ftc.teamcode.hardware

import com.acmerobotics.roadrunner.util.Angle
import com.outoftheboxrobotics.suspendftc.loopYieldWhile
import com.qualcomm.robotcore.hardware.IMU
import kotlinx.coroutines.Job
import kotlinx.coroutines.coroutineScope
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.isActive
import kotlinx.coroutines.launch
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.Globals
import kotlin.coroutines.CoroutineContext

sealed class IMUHandler {
    abstract val rawAngle: StateFlow<Double>
    private val angleOffset = MutableStateFlow(0.0)

    val angle get() = Angle.normDelta(rawAngle.value - angleOffset.value)

    fun resetAngle(newAngle: Double = 0.0) {
        angleOffset.value = rawAngle.value - newAngle
    }
}

class ThreadedImuHandler : IMUHandler() {
    override val rawAngle = MutableStateFlow(0.0)

    private lateinit var job: Job

    suspend fun runHandler(context: CoroutineContext, imu: IMU) = coroutineScope {
        job = launch(context) {
            while (isActive) {
                rawAngle.value = imu.robotYawPitchRollAngles.getYaw(AngleUnit.RADIANS)

                Globals.log.imu["raw_angle"] = rawAngle.value
                Globals.log.imu["corrected_angle"] = angle

                Globals.log.imu.collect()
            }
        }
    }

    fun cancel() = job.cancel()
}
class DefaultImuHandler : IMUHandler() {
    override val rawAngle = MutableStateFlow(0.0)

    suspend fun runHandler(imu: IMU) = coroutineScope {
        loopYieldWhile({ isActive }) {
            rawAngle.value = imu.robotYawPitchRollAngles.getYaw(AngleUnit.RADIANS)

            Globals.log.imu["raw_angle"] = rawAngle.value
            Globals.log.imu["corrected_angle"] = angle

            Globals.log.imu.collect()
        }
    }
}
