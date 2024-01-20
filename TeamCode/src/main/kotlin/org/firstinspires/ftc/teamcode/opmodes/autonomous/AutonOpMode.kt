package org.firstinspires.ftc.teamcode.opmodes.autonomous

import com.outoftheboxrobotics.suspendftc.suspendUntil
import kotlinx.coroutines.coroutineScope
import kotlinx.coroutines.launch
import org.firstinspires.ftc.teamcode.RobotState
import org.firstinspires.ftc.teamcode.actions.hardware.ArmPosition
import org.firstinspires.ftc.teamcode.actions.hardware.DriveControlState
import org.firstinspires.ftc.teamcode.actions.hardware.IntakeTiltPosition
import org.firstinspires.ftc.teamcode.actions.hardware.TwistPosition
import org.firstinspires.ftc.teamcode.actions.hardware.closeClaws
import org.firstinspires.ftc.teamcode.actions.hardware.currentDrivePose
import org.firstinspires.ftc.teamcode.actions.hardware.driveControlState
import org.firstinspires.ftc.teamcode.actions.hardware.launchFixpoint
import org.firstinspires.ftc.teamcode.actions.hardware.openClaws
import org.firstinspires.ftc.teamcode.actions.hardware.setArmPosition
import org.firstinspires.ftc.teamcode.actions.hardware.setDrivePowers
import org.firstinspires.ftc.teamcode.actions.hardware.setDrivetrainIdle
import org.firstinspires.ftc.teamcode.actions.hardware.setTiltPosition
import org.firstinspires.ftc.teamcode.actions.hardware.setTwistPosition
import org.firstinspires.ftc.teamcode.driveState
import org.firstinspires.ftc.teamcode.opmodes.RobotOpMode
import org.firstinspires.ftc.teamcode.util.C
import org.firstinspires.ftc.teamcode.util.G
import org.firstinspires.ftc.teamcode.util.mainLoop
import org.firstinspires.ftc.teamcode.util.set
import org.firstinspires.ftc.teamcode.util.suspendUntilRisingEdge
import org.firstinspires.ftc.teamcode.vision.PreloadDetectionPipeline
import org.firstinspires.ftc.teamcode.vision.preloadPosition
import org.firstinspires.ftc.teamcode.visionState

/**
 * Base class for autonomous op modes.
 *
 * @param isBlue Whether the robot is on the blue alliance.
 */
abstract class AutonOpMode(protected val isBlue: Boolean) : RobotOpMode() {
    private var enableBreakpoints = false

    /**
     * Runs the autonomous init routine. Should be cancelled manually on start.
     */
    suspend fun runAutonInit(): Nothing = coroutineScope {
        enableBreakpoints = false

        val cameraJob = launch {
            G.chub.outtakeCamera.let { webcam ->
                webcam.startCamera(640, 480)
                streamCamera(webcam)
                webcam.setPipeline(
                    PreloadDetectionPipeline().also { if (isBlue) it.setBlue() else it.setRed() }
                )
            }
        }

        setTwistPosition(TwistPosition.STRAIGHT)
        setArmPosition(ArmPosition.AUTON_INIT)
        setTiltPosition(IntakeTiltPosition.HIGH)

        launch {
            while (true) {
                suspendUntilRisingEdge { C.toggleAutoBreakpointEnable }
                enableBreakpoints = !enableBreakpoints
            }
        }

        mainLoop {
            if (C.openClaw) openClaws() else closeClaws()

            if (enableBreakpoints) {
                telemetry.addLine("BREAKPOINTS ARE ON")
            }

            if (cameraJob.isCompleted) {
                telemetry.addLine("Camera Ready")
                telemetry["Preload Position"] = G[RobotState.visionState.preloadPosition]
            } else {
                telemetry.addLine("Initializing Camera")
            }
        }
    }

    /**
     * Interrupts autonomous if breakpoints are enabled.
     *
     * Do not call while the drivetrain is in motion or being controlled by a concurrent coroutine.
     */
    suspend fun breakpoint() {
        if (!enableBreakpoints) return

        val lastPose = currentDrivePose()
        val wasFixpoint = G[RobotState.driveState.driveControlState] is DriveControlState.Fixpoint

        coroutineScope {
            val telemetryJob = launch {
                mainLoop { telemetry.addLine("BREAKPOINT") }
            }

            setDrivetrainIdle()
            setDrivePowers(0.0, 0.0, 0.0)

            suspendUntil { gamepad1.x }
            launchFixpoint(lastPose)
            suspendUntil { gamepad1.b }
            if (!wasFixpoint) setDrivetrainIdle()
            telemetryJob.cancel()
        }
    }
}