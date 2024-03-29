package org.firstinspires.ftc.teamcode.opmodes.testing

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.outoftheboxrobotics.suspendftc.suspendFor
import com.outoftheboxrobotics.suspendftc.suspendUntil
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import kotlinx.coroutines.cancelAndJoin
import kotlinx.coroutines.coroutineScope
import kotlinx.coroutines.flow.drop
import kotlinx.coroutines.flow.filterNotNull
import kotlinx.coroutines.flow.first
import kotlinx.coroutines.launch
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl
import org.firstinspires.ftc.teamcode.RobotState
import org.firstinspires.ftc.teamcode.actions.hardware.ArmPosition
import org.firstinspires.ftc.teamcode.actions.hardware.IntakeTiltPosition
import org.firstinspires.ftc.teamcode.actions.hardware.currentDrivePose
import org.firstinspires.ftc.teamcode.actions.hardware.dualFixpoint
import org.firstinspires.ftc.teamcode.actions.hardware.extendoPose
import org.firstinspires.ftc.teamcode.actions.hardware.ezStopExtension
import org.firstinspires.ftc.teamcode.actions.hardware.intakeTransfer
import org.firstinspires.ftc.teamcode.actions.hardware.launchFixpoint
import org.firstinspires.ftc.teamcode.actions.hardware.lineTo
import org.firstinspires.ftc.teamcode.actions.hardware.resetExtensionLength
import org.firstinspires.ftc.teamcode.actions.hardware.retractExtension
import org.firstinspires.ftc.teamcode.actions.hardware.runExtensionTo
import org.firstinspires.ftc.teamcode.actions.hardware.setArmPosition
import org.firstinspires.ftc.teamcode.actions.hardware.setDrivePowers
import org.firstinspires.ftc.teamcode.actions.hardware.setExtensionPower
import org.firstinspires.ftc.teamcode.actions.hardware.setTiltPosition
import org.firstinspires.ftc.teamcode.driveState
import org.firstinspires.ftc.teamcode.opmodes.RobotOpMode
import org.firstinspires.ftc.teamcode.opmodes.autonomous.AutonOpMode
import org.firstinspires.ftc.teamcode.util.G
import org.firstinspires.ftc.teamcode.util.mainLoop
import org.firstinspires.ftc.teamcode.util.mapState
import org.firstinspires.ftc.teamcode.util.next
import org.firstinspires.ftc.teamcode.util.use
import org.firstinspires.ftc.teamcode.vision.stackTapePose
import org.firstinspires.ftc.teamcode.visionState
import java.util.concurrent.TimeUnit
import kotlin.math.PI

@Autonomous
@Config
class SwoopTest : AutonOpMode(true, true) {
    companion object {
        @JvmField var robotTargetX = 50.0
        @JvmField var intakeTargetX = 86.0
        @JvmField var tapeEstimateX = 101.5

        @JvmField var intakeConstraint = 5.0
    }

    override suspend fun runSuspendOpMode() = coroutineScope {
        startTapeDetection()

        retractExtension()

        awaitCameraOpen()

        tapeDetectionPipeline.estimate = Vector2d(tapeEstimateX, 0.0)

        launch {
            mainLoop { telemetry.addLine("Ready") }
        }.use {
            suspendUntilStart()
        }

        val intakePos = launch {
            dualFixpoint(
                robotTarget = Vector2d(robotTargetX, 0.0),
                intakeTarget = Vector2d(intakeTargetX, 0.0),
                intakeHeadingConstraintMultiplier = intakeConstraint
            )
        }.use {
            suspendUntil { tapeDetectionPipeline.inFrame }

            suspendFor(2000)

            val tapePose = G.robotState.mapState { it.visionState.stackTapePose }.filterNotNull().drop(1).first()

            tapeDetectionPipeline.saveFrameState = true

            tapePose + Pose2d(
                (tapePose - currentDrivePose()).vec().rotated(PI / 2).let { it / it.norm() },
                0.0
            )
        }

        setTiltPosition(IntakeTiltPosition.POS_2)
        G.ehub.intakeRoller.power = -0.7

        launch {
            dualFixpoint(
                robotTarget = Vector2d(robotTargetX, 0.0),
                intakeTarget = intakePos.vec(),
                intakeHeadingConstraintMultiplier = 10.0
            )
        }.use {
            suspendUntil { (G[RobotState.driveState.extendoPose] - intakePos).vec().norm() < 1.0 }
        }

        setDrivePowers(Pose2d())

        G.ehub.intakeWheel.power = 0.0

        setExtensionPower(1.0)
        suspendFor(500)
        ezStopExtension()

        setTiltPosition(IntakeTiltPosition.POS_3)
        suspendFor(80)

        intakeTransfer()
    }
}