package org.firstinspires.ftc.teamcode.opmodes.testing

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.outoftheboxrobotics.suspendftc.suspendUntil
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import kotlinx.coroutines.launch
import org.firstinspires.ftc.teamcode.RobotState
import org.firstinspires.ftc.teamcode.actions.controllers.PidCoefs
import org.firstinspires.ftc.teamcode.actions.controllers.runPidController
import org.firstinspires.ftc.teamcode.actions.hardware.DriveConfig
import org.firstinspires.ftc.teamcode.actions.hardware.ExtensionConfig
import org.firstinspires.ftc.teamcode.actions.hardware.currentDrivePose
import org.firstinspires.ftc.teamcode.actions.hardware.extendoPose
import org.firstinspires.ftc.teamcode.actions.hardware.extensionLength
import org.firstinspires.ftc.teamcode.actions.hardware.setDrivePowers
import org.firstinspires.ftc.teamcode.actions.hardware.setExtensionPower
import org.firstinspires.ftc.teamcode.driveState
import org.firstinspires.ftc.teamcode.opmodes.RobotOpMode
import org.firstinspires.ftc.teamcode.runStateMachine
import org.firstinspires.ftc.teamcode.util.C
import org.firstinspires.ftc.teamcode.util.FS
import org.firstinspires.ftc.teamcode.util.G
import org.firstinspires.ftc.teamcode.util.cross
import org.firstinspires.ftc.teamcode.util.mainLoop
import org.firstinspires.ftc.teamcode.util.use

@TeleOp
@Config
class DoubleFixpointTest : RobotOpMode() {
    companion object {
        @JvmField var intakeWheel_kP = 1.0

        @JvmField var targetX = 0.0
        @JvmField var targetY = 0.0
    }

    private val driveState: FS = FS {
        launch {
            mainLoop {
                setDrivePowers(
                    C.driveStrafeX,
                    C.driveStrafeY,
                    C.driveTurn
                )

                setExtensionPower(-gamepad1.right_stick_y.toDouble())

                G.chub.intakeWheel.power = C.driveTurn
            }
        }.use {
            suspendUntil { gamepad1.x }
            pidState
        }
    }

    private val pidState: FS = FS {
        launch {
            var parError = 0.0
            var perpError = 0.0

            var turnPower = 0.0

            launch {
                mainLoop {
                    val target = Vector2d(targetX, targetY)
                    val extendoPose = G[RobotState.driveState.extendoPose]
                    val drivePose = currentDrivePose()

                    parError = (target - extendoPose.vec()) dot (target - drivePose.vec()).let { it / it.norm() }
                    perpError = (target - extendoPose.vec()) cross (target - drivePose.vec()).let { it / it.norm() }
                }
            }

            launch {
                runPidController(
                    coefs = PidCoefs(intakeWheel_kP, 0.0, 0.0),
                    input = { 0.0 },
                    target = { perpError },
                    output = { G.chub.intakeWheel.power = -it },
                    hz = 30
                )
            }

            launch {
                runPidController(
                    coefs = ExtensionConfig.pidCoefs,
                    input = { 0.0 },
                    target = { parError / DriveConfig.intakeOdoExtensionMultiplier },
                    output = { setExtensionPower(it) },
                    hz = 30
                )
            }

            launch {
                runPidController(
                    coefs = DriveConfig.headingPid,
                    input = { 0.0 },
                    target = { -perpError / (extensionLength() * DriveConfig.intakeOdoExtensionMultiplier + DriveConfig.intakeOdoRadius) },
                    output = { turnPower = it },
                    hz = 30
                )
            }

            launch {
                mainLoop {
                    setDrivePowers(
                        C.driveStrafeX,
                        C.driveStrafeY,
                        turnPower
                    )
                }
            }
        }.use {
            suspendUntil { gamepad1.y }
            driveState
        }
    }

    override suspend fun runSuspendOpMode() {
        suspendUntilStart()

        runStateMachine(driveState)
    }
}