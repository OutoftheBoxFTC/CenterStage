package org.firstinspires.ftc.teamcode.opmodes.testing

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.outoftheboxrobotics.suspendftc.suspendFor
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import kotlinx.coroutines.cancelAndJoin
import kotlinx.coroutines.coroutineScope
import kotlinx.coroutines.launch
import org.firstinspires.ftc.teamcode.actions.hardware.ArmPosition
import org.firstinspires.ftc.teamcode.actions.hardware.IntakeTiltPosition
import org.firstinspires.ftc.teamcode.actions.hardware.launchFixpoint
import org.firstinspires.ftc.teamcode.actions.hardware.lineTo
import org.firstinspires.ftc.teamcode.actions.hardware.resetExtensionLength
import org.firstinspires.ftc.teamcode.actions.hardware.runExtensionTo
import org.firstinspires.ftc.teamcode.actions.hardware.setArmPosition
import org.firstinspires.ftc.teamcode.actions.hardware.setTiltPosition
import org.firstinspires.ftc.teamcode.opmodes.RobotOpMode
import org.firstinspires.ftc.teamcode.util.G
import org.firstinspires.ftc.teamcode.util.mainLoop

@Autonomous
class SwoopTest : RobotOpMode() {
    override suspend fun runSuspendOpMode() = coroutineScope {
        setTiltPosition(IntakeTiltPosition.HIGH)
        setArmPosition(ArmPosition.NEUTRAL)
        resetExtensionLength()

        val targetPose = Pose2d(27.0, 0.0, 0.0)

        suspendUntilStart()

        val driveJob = launch {
            var driveMultiplier = 1.0

            val currentMonitor = launch {
                mainLoop {
                    G.ehub.readCurrents()
                    G.chub.readCurrents()

                    val driveCurrentBudget = 19.8 - G.ehub.hubCurrent
                    val driveCurrent = G.chub.hubCurrent

                    driveMultiplier =
                        (driveMultiplier * driveCurrentBudget / driveCurrent)
                            .coerceIn(0.0..1.0)

                    suspendFor(200)
                }
            }

            lineTo(
                targetPose,
                maxPower = { driveMultiplier }
            )

            launchFixpoint(targetPose)
            currentMonitor.cancelAndJoin()
        }

        runExtensionTo(1000, keepPid = true)

        driveJob.join()

        mainLoop {
            telemetry.addLine("Finished")
        }
    }
}