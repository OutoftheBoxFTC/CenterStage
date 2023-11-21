package org.firstinspires.ftc.teamcode.opmodes

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
import org.firstinspires.ftc.teamcode.actions.hardware.runDefaultImuHandler
import org.firstinspires.ftc.teamcode.actions.hardware.runThreadedImuHandler
import org.firstinspires.ftc.teamcode.driveLooper
import org.firstinspires.ftc.teamcode.drivetrainHandler
import org.firstinspires.ftc.teamcode.hardware.IMU_NAME
import org.firstinspires.ftc.teamcode.mainLooper
import org.firstinspires.ftc.teamcode.subsystems.RoadrunnerDrivetrain

abstract class RobotOpMode(
    private val runMultiThreaded: Boolean = true,
    private val monitorOpmodeStop: Boolean = true,
    private val imuRunMode: ImuRunMode = ImuRunMode.THREADED
) : LinearOpMode() {
    enum class ImuRunMode {
        NONE,
        DEFAULT,
        THREADED
    }

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

        val mainLooperLens = RobotState.mainLooper
        val driveLooperLens = RobotState.driveLooper

        defaultRobotState(hardwareMap, telemetry).copy {
            if (runMultiThreaded) { driveLooperLens set Looper() }
        }.let { Globals.initializeRobotState(it, this) }

        (Globals[RobotState.drivetrainHandler] as? RoadrunnerDrivetrain)?.initialize()

        Globals[driveLooperLens].scheduleCoroutine {
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
                if (imuRunMode != ImuRunMode.NONE) {
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
                        when (imuRunMode) {
                            ImuRunMode.DEFAULT -> runDefaultImuHandler(imu)
                            ImuRunMode.THREADED -> runThreadedImuHandler(imuThread.bind(), imu)
                            else -> error("IMU Initialization in RobotOpMode")
                        }
                    }
                }

                Globals[driveLooperLens].let {
                    launch(driveThread.bind()) {
                        it.run()
                    }
                }

                Globals[mainLooperLens].run()
            }
        }
    }
}