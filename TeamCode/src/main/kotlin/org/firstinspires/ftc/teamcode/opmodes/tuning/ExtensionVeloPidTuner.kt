package org.firstinspires.ftc.teamcode.opmodes.tuning

import arrow.core.merge
import arrow.fx.coroutines.raceN
import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import kotlinx.coroutines.coroutineScope
import kotlinx.coroutines.launch
import org.firstinspires.ftc.teamcode.actions.controllers.FeedforwardCoefs
import org.firstinspires.ftc.teamcode.actions.controllers.PidCoefs
import org.firstinspires.ftc.teamcode.actions.controllers.runVeloPid
import org.firstinspires.ftc.teamcode.actions.hardware.extensionLength
import org.firstinspires.ftc.teamcode.opmodes.RobotOpMode
import org.firstinspires.ftc.teamcode.runStateMachine
import org.firstinspires.ftc.teamcode.util.FS
import org.firstinspires.ftc.teamcode.util.G
import org.firstinspires.ftc.teamcode.util.cancellationScope
import org.firstinspires.ftc.teamcode.util.mainLoop
import org.firstinspires.ftc.teamcode.util.set
import org.firstinspires.ftc.teamcode.util.suspendUntilRisingEdge
import org.firstinspires.ftc.teamcode.util.use

@TeleOp
@Config
class ExtensionVeloPidTuner : RobotOpMode() {
    companion object {
        @JvmField var kP = 0.0001
        @JvmField var kI = 0.002
        @JvmField var kD = 0.000005

        @JvmField var kV = 0.001
        @JvmField var kA = 0.00005

        @JvmField var minPos = 100
        @JvmField var maxPos = 1200
        @JvmField var maxVel = 1500.0
        @JvmField var maxAccel = 3000.0
    }

    private val pidCoefs = PidCoefs(0.0, 0.0, 0.0)
    private val feedforwardCoefs = FeedforwardCoefs(0.0, 0.0, 0.0, 0.0)

    private fun updateCoefs() {
        pidCoefs.run {
            kP = Companion.kP
            kI = Companion.kI
            kD = Companion.kD
        }

        feedforwardCoefs.run {
            kV = Companion.kV
            kA = Companion.kA
        }
    }

    private val manualControl: FS = FS {
        launch {
            mainLoop {
                G.ehub.extension.power = -gamepad1.left_stick_y.toDouble()

                telemetry["vel"] = G.ehub.extension.velocity
                telemetry["kV"] = G.ehub.extension
                    .let { it.power / it.velocity }
                    .let { if (G.ehub.extension.velocity < maxVel / 2) 0.0 else it }
            }
        }.use {
            suspendUntilRisingEdge { gamepad1.x }

            pidState
        }
    }

    private val pidState: FS = FS {
        cancellationScope {
            var target = minPos
            var targetVel = 0.0

            launch {
                runVeloPid(
                    feedforward = feedforwardCoefs,
                    pid = pidCoefs,
                    input = { extensionLength().toDouble() },
                    target = { target.toDouble() },
                    output = { G.ehub.extension.power = it },
                    maxVel = maxVel,
                    maxAccel = maxAccel,
                    hz = 30,

                    targetMotionStateDebugging = { targetVel = it.v }
                )
            }

            launch {
                while (true) {
                    target = raceN(
                        coroutineContext,
                        {
                            suspendUntilRisingEdge { gamepad1.left_bumper }
                            maxPos
                        },
                        {
                            suspendUntilRisingEdge { gamepad1.right_bumper }
                            minPos
                        }
                    ).merge()
                }
            }

            launch {
                mainLoop {
                    telemetry["pos"] = extensionLength()
                    telemetry["vel"] = G.ehub.extension.velocity
                    telemetry["target vel"] = targetVel
                }
            }

            suspendUntilRisingEdge { gamepad1.y }

            manualControl
        }
    }

    override suspend fun runSuspendOpMode() {
        suspendUntilStart()

        coroutineScope {
            launch { runStateMachine(manualControl) }

            mainLoop {
                updateCoefs()
            }
        }
    }
}