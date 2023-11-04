package org.firstinspires.ftc.teamcode.opmodes

import arrow.core.Option
import arrow.core.none
import arrow.fx.coroutines.autoCloseable
import arrow.fx.coroutines.resourceScope
import arrow.optics.copy
import com.outoftheboxrobotics.suspendftc.Looper
import com.outoftheboxrobotics.suspendftc.loopYieldWhile
import com.outoftheboxrobotics.suspendftc.suspendUntil
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.IMU
import kotlinx.coroutines.DelicateCoroutinesApi
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.ExperimentalCoroutinesApi
import kotlinx.coroutines.isActive
import kotlinx.coroutines.launch
import kotlinx.coroutines.newSingleThreadContext
import kotlinx.coroutines.runBlocking
import kotlinx.coroutines.withContext
import org.firstinspires.ftc.teamcode.Globals
import org.firstinspires.ftc.teamcode.Globals.defaultRobotState
import org.firstinspires.ftc.teamcode.RobotState
import org.firstinspires.ftc.teamcode.command.CommandHandler
import org.firstinspires.ftc.teamcode.commandHandler
import org.firstinspires.ftc.teamcode.driveLooper
import org.firstinspires.ftc.teamcode.hardware.DefaultImuHandler
import org.firstinspires.ftc.teamcode.hardware.IMUHandler
import org.firstinspires.ftc.teamcode.hardware.IMU_NAME
import org.firstinspires.ftc.teamcode.hardware.ThreadedImuHandler
import org.firstinspires.ftc.teamcode.imuHandler
import org.firstinspires.ftc.teamcode.looper
import org.firstinspires.ftc.teamcode.mainLooper

abstract class RobotOpMode(
    private val runMultiThreaded: Boolean = false,
    private val monitorOpmodeStop: Boolean = true,
    private val imuHandler: Option<IMUHandler> = none()
) : LinearOpMode() {
    abstract suspend fun runSuspendOpMode()

    protected suspend fun suspendUntilStart() {
        withContext(Dispatchers.IO) {
            waitForStart()
        }
    }

    @OptIn(ExperimentalCoroutinesApi::class, DelicateCoroutinesApi::class)
    override fun runOpMode() {
        val mainLooperLens = RobotState.looper.mainLooper
        val driveLooperLens = RobotState.looper.driveLooper

        defaultRobotState(hardwareMap).copy {
            if (runMultiThreaded) { driveLooperLens set Looper() }
            RobotState.imuHandler set imuHandler
        }.let { Globals.initializeRobotState(it, this) }

        (Globals[driveLooperLens] ?: Globals[mainLooperLens])
            .scheduleCoroutine {
                loopYieldWhile({ isActive }) {
                    Globals.chub.syncHardware()
                }
            }

        Globals[mainLooperLens].scheduleCoroutine {
            loopYieldWhile({ isActive }) {
                Globals.ehub.syncHardware()
            }
        }

        Globals[mainLooperLens].scheduleCoroutine {
            val job = launch { runSuspendOpMode() }

            suspendUntil {
                job.isCompleted || (monitorOpmodeStop && !opModeIsActive() && !opModeInInit())
            }

            Globals.stop()
        }

        val driveThread = autoCloseable { newSingleThreadContext("Drive thread") }
        val imuThread = autoCloseable { newSingleThreadContext("IMU thread") }

        runBlocking {
            Globals[RobotState.commandHandler] = CommandHandler.new()

            resourceScope {
                val imuHandler = Globals[RobotState.imuHandler].getOrNull()

                if (imuHandler != null) {
                    val imu = hardwareMap[IMU::class.java, IMU_NAME]

                    when (imuHandler) {
                        is DefaultImuHandler -> imuHandler.startHandler(
                            Globals[mainLooperLens],
                            imu
                        )

                        is ThreadedImuHandler -> Globals[mainLooperLens].scheduleCoroutine {
                            imuHandler.runHandler(imuThread.bind(), imu)
                        }
                    }
                }

                Globals[driveLooperLens]?.let {
                    launch(driveThread.bind()) {
                        it.run()
                    }
                }

                Globals[mainLooperLens].run()
            }
        }
    }
}