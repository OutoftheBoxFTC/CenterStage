package org.firstinspires.ftc.teamcode.hardware

import com.outoftheboxrobotics.suspendftc.Looper
import com.outoftheboxrobotics.suspendftc.loopYieldWhile
import com.qualcomm.robotcore.hardware.IMU
import kotlinx.coroutines.Job
import kotlinx.coroutines.coroutineScope
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.isActive
import kotlinx.coroutines.launch
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import kotlin.coroutines.CoroutineContext

sealed interface IMUHandler {
    val angle: StateFlow<Double>
}

class ThreadedImuHandler : IMUHandler {
    override val angle = MutableStateFlow(0.0)

    private lateinit var job: Job

    suspend fun runHandler(context: CoroutineContext, imu: IMU) = coroutineScope {
        job = launch(context) {
            while (isActive) {
                angle.value = imu.robotYawPitchRollAngles.getYaw(AngleUnit.RADIANS)
            }
        }
    }

    fun cancel() = job.cancel()
}
class DefaultImuHandler : IMUHandler {
    override val angle = MutableStateFlow(0.0)

    fun startHandler(looper: Looper, imu: IMU) {
        looper.scheduleCoroutine {
            loopYieldWhile({ isActive }) {
                angle.value = imu.robotYawPitchRollAngles.getYaw(AngleUnit.RADIANS)
            }
        }
    }
}
