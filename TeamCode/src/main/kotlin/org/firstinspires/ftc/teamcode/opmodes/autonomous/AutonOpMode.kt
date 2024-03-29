package org.firstinspires.ftc.teamcode.opmodes.autonomous

import arrow.core.toOption
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.outoftheboxrobotics.suspendftc.joinAndYield
import com.outoftheboxrobotics.suspendftc.suspendUntil
import kotlinx.coroutines.Job
import kotlinx.coroutines.coroutineScope
import kotlinx.coroutines.launch
import org.firstinspires.ftc.teamcode.RobotState
import org.firstinspires.ftc.teamcode.actions.hardware.ArmPosition
import org.firstinspires.ftc.teamcode.actions.hardware.DriveControlState
import org.firstinspires.ftc.teamcode.actions.hardware.IntakeTiltPosition
import org.firstinspires.ftc.teamcode.actions.hardware.TwistPosition
import org.firstinspires.ftc.teamcode.actions.hardware.closeDrone
import org.firstinspires.ftc.teamcode.actions.hardware.currentDrivePose
import org.firstinspires.ftc.teamcode.actions.hardware.driveControlState
import org.firstinspires.ftc.teamcode.actions.hardware.launchSmoothStop
import org.firstinspires.ftc.teamcode.actions.hardware.openClaws
import org.firstinspires.ftc.teamcode.actions.hardware.resetDrivePose
import org.firstinspires.ftc.teamcode.actions.hardware.resetImuAngle
import org.firstinspires.ftc.teamcode.actions.hardware.setArmPosition
import org.firstinspires.ftc.teamcode.actions.hardware.setDrivePowers
import org.firstinspires.ftc.teamcode.actions.hardware.setDrivetrainIdle
import org.firstinspires.ftc.teamcode.actions.hardware.setTiltPosition
import org.firstinspires.ftc.teamcode.actions.hardware.setTwistPosition
import org.firstinspires.ftc.teamcode.driveState
import org.firstinspires.ftc.teamcode.hardware.devices.KWebcam
import org.firstinspires.ftc.teamcode.mainLooper
import org.firstinspires.ftc.teamcode.opmodes.RobotOpMode
import org.firstinspires.ftc.teamcode.util.C
import org.firstinspires.ftc.teamcode.util.G
import org.firstinspires.ftc.teamcode.util.mainLoop
import org.firstinspires.ftc.teamcode.util.set
import org.firstinspires.ftc.teamcode.util.suspendUntilRisingEdge
import org.firstinspires.ftc.teamcode.util.use
import org.firstinspires.ftc.teamcode.vision.AprilTagDetectionPipeline
import org.firstinspires.ftc.teamcode.vision.PreloadDetectionPipeline
import org.firstinspires.ftc.teamcode.vision.TapeDetectionPipeline
import org.firstinspires.ftc.teamcode.vision.preloadPosition
import org.firstinspires.ftc.teamcode.visionState
import org.openftc.easyopencv.OpenCvSwitchableWebcam
import kotlin.properties.Delegates

/**
 * Base class for autonomous op modes.
 *
 * @param isBlue Whether the robot is on the blue alliance.
 */
abstract class AutonOpMode(protected val isBlue: Boolean, private val isAud: Boolean) : RobotOpMode() {
    private var enableBreakpoints = false

    private var activeCamera: KWebcam? = null
    private var cameraOpenJob: Job? = null

    val preloadDetectionPipeline = PreloadDetectionPipeline().also {
        when {
            isBlue && isAud -> it.setBlueAud()
            isBlue && !isAud -> it.setBlueBack()
            !isBlue && isAud -> it.setRedAud()
            else -> it.setRedBack()
        }
    }

    val outtakeAprilTagPipeline = AprilTagDetectionPipeline.outtakePipeline()
    val tapeDetectionPipeline = TapeDetectionPipeline()

    /**
     * Runs the autonomous init routine. Should be cancelled manually on start.
     */
    suspend fun runAutonInit(): Nothing = coroutineScope {
        enableBreakpoints = false

        val cameraJob = launch {
            startPreloadDetection()
            awaitCameraOpen()

            resetImuAngle()
            resetDrivePose()
        }

        setTwistPosition(TwistPosition.STRAIGHT)
        setArmPosition(ArmPosition.AUTON_INIT)
        setTiltPosition(IntakeTiltPosition.TRANSFER)
        closeDrone()
        openClaws()

        launch {
            while (true) {
                suspendUntilRisingEdge { C.toggleAutoBreakpointEnable }
                enableBreakpoints = !enableBreakpoints
            }
        }

        mainLoop {
            if (enableBreakpoints) {
                telemetry.addLine("BREAKPOINTS ARE ON")
            }

            if (cameraJob.isCompleted) {
                telemetry.addLine("Camera Ready")
                telemetry["Preload Position"] = G[RobotState.visionState.preloadPosition]
            } else {
                telemetry.addLine("Initializing Camera")
            }

            setTiltPosition(
                if (C.openTiltAuto) IntakeTiltPosition.TRANSFER
                else IntakeTiltPosition.PRELOAD_HOLD
            )

            if (C.resetAutoPose) {
                resetImuAngle()
                resetDrivePose()
            }
        }
    }

    suspend fun awaitCameraOpen() = cameraOpenJob?.joinAndYield() ?: error("No Camera currently being opened")

    fun startPreloadDetection() {
        cameraOpenJob = G[RobotState.mainLooper].scheduleCoroutine {
            G.chub.outtakeCamera.let {
                if (activeCamera != it) {
                    activeCamera?.stopStreaming()
                    activeCamera = it
                    it.startCamera(640, 480)
                    streamCamera(it)
                }

                it.setPipeline(preloadDetectionPipeline)
            }
        }
    }

    fun startApriltagDetection() {
        cameraOpenJob = G[RobotState.mainLooper].scheduleCoroutine {
            G.chub.outtakeCamera.let {
                if (activeCamera != it) {
                    activeCamera?.stopStreaming()
                    activeCamera = it
                    it.startCamera(640, 480)
                    streamCamera(it)
                }

                it.setPipeline(outtakeAprilTagPipeline)
            }
        }
    }

    fun startTapeDetection() {
        cameraOpenJob = G[RobotState.mainLooper].scheduleCoroutine {
            G.chub.intakeCamera.let {
                if (activeCamera != it) {
                    activeCamera?.stopStreaming()
                    activeCamera = it
                    it.startCamera(1280, 800)
                    streamCamera(it)
                }

                it.setPipeline(tapeDetectionPipeline)
            }
        }
    }

    /**
     * Interrupts autonomous if breakpoints are enabled.
     *
     * Do not call while the drivetrain is in motion or being controlled by a concurrent coroutine.
     */
    suspend fun breakpoint() = coroutineScope {
        if (!enableBreakpoints || !gamepad1.a) return@coroutineScope

        var wasFixpoint by Delegates.notNull<Boolean>()

        val fixpointTarget =
            (G[RobotState.driveState.driveControlState] as? DriveControlState.Fixpoint)
                .toOption()
                .fold(
                    ifEmpty = { wasFixpoint = false; currentDrivePose() },
                    ifSome = { wasFixpoint = true; it.target }
                )


        launch {
            mainLoop { telemetry.addLine("BREAKPOINT") }
        }.use {
            if (wasFixpoint) {
                suspendUntilRisingEdge { gamepad1.x }

                setDrivetrainIdle()
                setDrivePowers(0.0, 0.0, 0.0)
            } else {
                setDrivetrainIdle()
                setDrivePowers(0.0, 0.0, 0.0)
            }

            suspendUntilRisingEdge { gamepad1.x }

            launchSmoothStop(fixpointTarget)

            suspendUntil { gamepad1.b }

            if (!wasFixpoint) {
                setDrivetrainIdle()
                setDrivePowers(0.0, 0.0, 0.0)
            }
        }
    }
}