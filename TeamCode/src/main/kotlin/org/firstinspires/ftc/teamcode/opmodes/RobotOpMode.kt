package org.firstinspires.ftc.teamcode.opmodes

import arrow.fx.coroutines.autoCloseable
import arrow.fx.coroutines.resourceScope
import arrow.optics.copy
import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.roadrunner.geometry.Pose2d
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
import org.firstinspires.ftc.teamcode.actions.hardware.DriveControlState
import org.firstinspires.ftc.teamcode.actions.hardware.resetDrivePose
import org.firstinspires.ftc.teamcode.actions.hardware.resetExtensionLength
import org.firstinspires.ftc.teamcode.actions.hardware.runDefaultImuHandler
import org.firstinspires.ftc.teamcode.actions.hardware.runRoadrunnerDrivetrain
import org.firstinspires.ftc.teamcode.actions.hardware.runThreadedImuHandler
import org.firstinspires.ftc.teamcode.driveLooper
import org.firstinspires.ftc.teamcode.driveState
import org.firstinspires.ftc.teamcode.hardware.IMU_NAME
import org.firstinspires.ftc.teamcode.hardware.devices.KWebcam
import org.firstinspires.ftc.teamcode.logState
import org.firstinspires.ftc.teamcode.mainLooper
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.util.G

/**
 * Base class for all OpModes.
 *
 * @param runMultiThreaded Whether to run the main and drive loops in separate threads.
 * @param monitorOpmodeStop Whether to stop the OpMode automatically when the driver station stops it.
 * @param imuRunMode Whether to run the IMU in the main thread, a separate thread, or not at all.
 * @param resetEncoders Whether to reset the encoders on init.
 * @param resetPoseOnStart Whether to reset the drive pose on start.
 */
abstract class RobotOpMode(
    private val runMultiThreaded: Boolean = true,
    private val monitorOpmodeStop: Boolean = true,
    private val imuRunMode: ImuRunMode = ImuRunMode.THREADED,
    private val resetEncoders: Boolean = false,
    private val resetPoseOnStart: Boolean = true
) : LinearOpMode() {
    enum class ImuRunMode {
        /**
         * Do not run the IMU.
         */
        NONE,

        /**
         * Run the IMU in the drive thread.
         */
        DEFAULT,

        /**
         * Run the IMU in its own thread.
         */
        THREADED
    }

    abstract suspend fun runSuspendOpMode()

    /**
     * Stops the OpMode if set to true.
     */
    protected var manualStop = false

    /**
     * Suspends until the OpMode is started.
     */
    protected suspend fun suspendUntilStart() {
        withContext(Dispatchers.IO) {
            waitForStart()
        }

        if (resetPoseOnStart) resetDrivePose(Pose2d())
    }

    /**
     * Streams the camera to FtcDashboard.
     */
    protected fun streamCamera(camera: KWebcam) =
        FtcDashboard.getInstance().startCameraStream(camera, 0.0)

    /**
     * Resets all encoder values.
     */
    private fun resetEncoders() {
        resetExtensionLength()
    }

    @OptIn(ExperimentalCoroutinesApi::class, DelicateCoroutinesApi::class)
    override fun runOpMode() {
        manualStop = false

        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
        telemetry.msTransmissionInterval = 15

        // Convenience variables
        val mainLooperLens = RobotState.mainLooper
        val driveLooperLens = RobotState.driveLooper

        // Initialize Globals with the default robot state.
        // Adds the drive looper if running multi-threaded.
        defaultRobotState(hardwareMap).copy {
            if (runMultiThreaded) { driveLooperLens set Looper() }
        }.let { Globals.initializeRobotState(it, this) }

        // Roadrunner SampleMecanumDrive
        val rrDrive = SampleMecanumDrive(G.chub)

        Globals[driveLooperLens].scheduleCoroutine {
            launch {
                runRoadrunnerDrivetrain(rrDrive)
            }

            // Hardware sync loop for control hub
            loopYieldWhile({ isActive }) {
                Globals.chub.syncHardware()
            }
        }

        Globals[mainLooperLens].scheduleCoroutine {
            loopYieldWhile({ isActive }) {
                // Hardware sync loop for expansion hub
                Globals.ehub.syncHardware()

                // Gather telemetry packet from RR drive
                val driveState = Globals[RobotState.driveState]

                driveState.rrDrive?.let {
                    val packet = it.getDriveTelemetryPacket(
                        (driveState.driveControlState as? DriveControlState.Fixpoint)?.target,
                        driveState.extendoPose
                    )

                    FtcDashboard.getInstance().sendTelemetryPacket(packet)
                }

                // Update telemetry with robot state data
                telemetry.logState(Globals.robotState.value)
                telemetry.update()
            }
        }

        Globals[mainLooperLens].scheduleCoroutine {
            // Main runSuspendOpMode job
            val job = launch { runSuspendOpMode() }

            // OpMode stop conditions
            suspendUntil {
                manualStop || job.isCompleted || (monitorOpmodeStop && !opModeIsActive() && !opModeInInit())
            }

            Globals.stop()
        }

        // Thread resources for use with arrow-fx Resource DSL
        val driveThread = autoCloseable { newSingleThreadContext("Drive thread") }
        val imuThread = autoCloseable { newSingleThreadContext("IMU thread") }

        if (resetEncoders) resetEncoders()

        runBlocking {
            // Resource DSL to make sure threads are closed when finished.
            resourceScope {
                if (imuRunMode != ImuRunMode.NONE) {
                    // Initialize IMU device
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

                    Globals[driveLooperLens].scheduleCoroutine {
                        when (imuRunMode) {
                            ImuRunMode.DEFAULT -> runDefaultImuHandler(imu)
                            // .bind() creates the imuThread resource bounded by the resource scope
                            ImuRunMode.THREADED -> runThreadedImuHandler(imuThread.bind(), imu)
                            else -> error("IMU Initialization in RobotOpMode")
                        }
                    }
                }

                // Run the drive looper in its own thread if multi-threaded
                if (runMultiThreaded) launch(driveThread.bind()) {
                    Globals[driveLooperLens].run()
                }

                // Run the main looper in this thread
                Globals[mainLooperLens].run()
            }
        }
    }
}