package org.firstinspires.ftc.teamcode.opmodes

import arrow.core.Option
import arrow.core.none
import arrow.fx.coroutines.autoCloseable
import arrow.fx.coroutines.resourceScope
import arrow.optics.copy
import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.outoftheboxrobotics.suspendftc.Looper
import com.outoftheboxrobotics.suspendftc.loopYieldWhile
import com.outoftheboxrobotics.suspendftc.suspendUntil
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
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
import org.firstinspires.ftc.teamcode.driveLooper
import org.firstinspires.ftc.teamcode.hardware.devices.DefaultImuHandler
import org.firstinspires.ftc.teamcode.hardware.devices.IMUHandler
import org.firstinspires.ftc.teamcode.hardware.IMU_NAME
import org.firstinspires.ftc.teamcode.hardware.devices.ThreadedImuHandler
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
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
        telemetry.msTransmissionInterval = 15

        val mainLooperLens = RobotState.looper.mainLooper
        val driveLooperLens = RobotState.looper.driveLooper

        defaultRobotState(hardwareMap, telemetry).copy {
            if (runMultiThreaded) { driveLooperLens set Looper() }
            RobotState.imuHandler set imuHandler
        }.let { Globals.initializeRobotState(it, this) }

        (Globals[driveLooperLens] ?: Globals[mainLooperLens])
            .scheduleCoroutine {
                launch {
                    Globals.drive.runHandler()
                }

                loopYieldWhile({ isActive }) {
                    Globals.chub.syncHardware()
                }
            }

        Globals[mainLooperLens].scheduleCoroutine {
            loopYieldWhile({ isActive }) {
                Globals.log.hardware.collect()
                Globals.log.rootLog.collect()
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
            resourceScope {
                val imuHandler = Globals[RobotState.imuHandler].getOrNull()

                if (imuHandler != null) {
                    val imu = hardwareMap[IMU::class.java, IMU_NAME]

                    imu.initialize(
                        IMU.Parameters(
                            RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
                            )
                        )
                    )

                    imu.resetYaw()

                    Globals[mainLooperLens].scheduleCoroutine {
                        when (imuHandler) {
                            is DefaultImuHandler -> imuHandler.runHandler(imu)
                            is ThreadedImuHandler -> imuHandler.runHandler(imuThread.bind(), imu)
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