package org.firstinspires.ftc.teamcode.opmodes.testing

import arrow.core.merge
import arrow.fx.coroutines.raceN
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.outoftheboxrobotics.suspendftc.suspendUntil
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import kotlinx.coroutines.launch
import org.firstinspires.ftc.teamcode.actions.hardware.dualFixpoint
import org.firstinspires.ftc.teamcode.actions.hardware.intakeFixpoint
import org.firstinspires.ftc.teamcode.actions.hardware.setDrivePowers
import org.firstinspires.ftc.teamcode.actions.hardware.setDrivetrainIdle
import org.firstinspires.ftc.teamcode.actions.hardware.setExtensionPower
import org.firstinspires.ftc.teamcode.opmodes.RobotOpMode
import org.firstinspires.ftc.teamcode.runStateMachine
import org.firstinspires.ftc.teamcode.util.C
import org.firstinspires.ftc.teamcode.util.FS
import org.firstinspires.ftc.teamcode.util.G
import org.firstinspires.ftc.teamcode.util.mainLoop
import org.firstinspires.ftc.teamcode.util.use

@TeleOp
@Config
class DoubleFixpointTest : RobotOpMode() {
    companion object {
        @JvmField var targetX = 0.0
        @JvmField var targetY = 0.0

        @JvmField var robotX = 0.0
        @JvmField var robotY = 0.0
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

                G.ehub.intakeWheel.power = C.driveTurn
            }
        }.use {
            raceN(
                coroutineContext,
                {
                    suspendUntil { gamepad1.x }
                    pidState
                },
                {
                    suspendUntil { gamepad1.b }
                    dualFixpointState
                }
            ).merge()
        }
    }

    private val pidState: FS = FS {
        launch {
            var turnPower = 0.0

            launch {
                intakeFixpoint(
                    target = { Vector2d(targetX, targetY) },
                    headingOutput = {
                        turnPower = it
                    }
                )
            }

            mainLoop {
                setDrivePowers(
                    C.driveStrafeX,
                    C.driveStrafeY,
                    turnPower
                )
            }
        }.use {
            suspendUntil { gamepad1.y }
            driveState
        }
    }

    private val dualFixpointState: FS = FS {
        launch {
            try {
                dualFixpoint(
                    intakeTarget = Vector2d(targetX, targetY),
                    robotTarget = Vector2d(robotX, robotY)
                )
            } finally {
                setDrivetrainIdle()
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