package org.firstinspires.ftc.teamcode.opmodes.testing

import com.outoftheboxrobotics.suspendftc.loopYieldWhile
import com.outoftheboxrobotics.suspendftc.suspendFor
import com.outoftheboxrobotics.suspendftc.suspendUntil
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import kotlinx.coroutines.coroutineScope
import kotlinx.coroutines.launch
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import org.firstinspires.ftc.teamcode.actions.hardware.retractExtension
import org.firstinspires.ftc.teamcode.actions.hardware.setExtensionPower
import org.firstinspires.ftc.teamcode.opmodes.RobotOpMode
import org.firstinspires.ftc.teamcode.util.G
import org.firstinspires.ftc.teamcode.util.mainLoop
import org.firstinspires.ftc.teamcode.util.set
import kotlin.math.sign

@TeleOp
@Disabled
class ExtensionRetractTest : RobotOpMode() {
    override suspend fun runSuspendOpMode(): Unit = coroutineScope {
        suspendUntilStart()

        launch {
            mainLoop {
                telemetry["limit switch"] = G.chub.extensionLimitSwitch
            }
        }

        while (true) {
            suspendUntil { gamepad1.y }
            retractExtension()
        }
    }
}

@TeleOp
@Disabled
class ExtensionTest : RobotOpMode() {
    override suspend fun runSuspendOpMode() {
        suspendUntilStart()

        coroutineScope {
            launch {
                loopYieldWhile({ true }) {
                    suspendFor(250)
                    G.ehub.readCurrents()
                }
            }

            loopYieldWhile({ true }) {
                setExtensionPower(-gamepad1.left_stick_y.toDouble())

                telemetry["encoder pos"] = G.ehub.extension.currentPosition
                telemetry["encoder vel"] = G.ehub.extension.velocity
                telemetry["current"] = G.ehub.extension.getCurrent(CurrentUnit.AMPS)
            }
        }
    }
}

@Autonomous
class ExtensionWheelPivotTest : RobotOpMode() {
    override suspend fun runSuspendOpMode() {
        suspendUntilStart()

        mainLoop {
            G.chub.run {
                intakeWheel.power = -sign(odoIntake.currentPosition.toDouble())
            }
        }
    }
}

@TeleOp
class ExtensionRamDetectionTest : RobotOpMode() {
    override suspend fun runSuspendOpMode() {
        suspendUntilStart()

        mainLoop {
            val power = -gamepad1.left_stick_y.toDouble()

            G.ehub.extension.power = power

            telemetry["extension kV"] = if (power != 0.0) G.ehub.extension.velocity / power else 0.0
        }
    }
}