package org.firstinspires.ftc.teamcode.opmodes.tuning

import arrow.core.merge
import arrow.core.nel
import arrow.core.some
import arrow.fx.coroutines.raceN
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.profile.MotionState
import com.outoftheboxrobotics.suspendftc.loopYieldWhile
import com.outoftheboxrobotics.suspendftc.suspendUntil
import kotlinx.coroutines.coroutineScope
import kotlinx.coroutines.launch
import kotlinx.coroutines.withTimeoutOrNull
import org.firstinspires.ftc.teamcode.actions.controllers.FeedforwardCoefs
import org.firstinspires.ftc.teamcode.actions.controllers.PidCoefs
import org.firstinspires.ftc.teamcode.actions.controllers.runPidController
import org.firstinspires.ftc.teamcode.command.Subsystem
import org.firstinspires.ftc.teamcode.hardware.devices.ThreadedImuHandler
import org.firstinspires.ftc.teamcode.opmodes.RobotOpMode
import org.firstinspires.ftc.teamcode.statemachine.runStateMachine
import org.firstinspires.ftc.teamcode.subsystems.misc.MotionProfileController
import org.firstinspires.ftc.teamcode.util.FS
import org.firstinspires.ftc.teamcode.util.G
import org.firstinspires.ftc.teamcode.util.launchCommand
import org.firstinspires.ftc.teamcode.util.set

@Config
abstract class PidfTuner(
    subsystem: Subsystem
) : RobotOpMode(
    runMultiThreaded = true,
    imuHandler = ThreadedImuHandler().some()
) {
    companion object {
        @JvmField var kP = 0.0
        @JvmField var kI = 0.0
        @JvmField var kD = 0.0

        @JvmField var kV = 0.0
        @JvmField var kA = 0.0
        @JvmField var kS = 0.0
        @JvmField var kG = 0.0

        @JvmField var minPos = 0.0
        @JvmField var maxPos = 0.0
        @JvmField var maxVel = 0.0
        @JvmField var maxAccel = 0.0
    }

    abstract fun updateOutput(output: Double)
    abstract fun readInput(): Double

    private val pidCoefs = PidCoefs(0.0, 0.0, 0.0)
    private val feedforwardCoefs = FeedforwardCoefs(0.0, 0.0, 0.0, 0.0)

    private var target = 0.0

    private fun updateCoefs() {
        pidCoefs.run {
            kP = Companion.kP
            kI = Companion.kI
            kD = Companion.kD
        }

        feedforwardCoefs.run {
            kV = Companion.kV
            kA = Companion.kA
            kS = Companion.kS
            kG = Companion.kG
        }
    }

    private suspend fun runPid(target: Double): Nothing = runPidController(
        coefs = pidCoefs,
        input = ::readInput,
        target = { target },
        output = ::updateOutput
    )


    private val freeState: FS = FS {
        G.cmd.launchCommand(subsystem.nel()) {
            updateOutput(feedforwardCoefs.kG)
        }

        raceN(
            coroutineContext,
            {
                suspendUntil { gamepad1.b }
                profileState
            },
            {
                suspendUntil { gamepad1.x }
                pidState
            }
        ).merge()
    }

    @Suppress("IMPLICIT_NOTHING_TYPE_ARGUMENT_IN_RETURN_POSITION")
    private val profileState: FS = FS {
        val profiler = object : MotionProfileController(
            feedforwardCoefs,
            pidCoefs,
            maxVel,
            maxAccel,
            subsystem,
            MotionState(readInput(), 0.0)
        ) {
            override fun feedback() = readInput()
            override fun updateOutput(output: Double) {

                this@PidfTuner.updateOutput(output)
            }
        }

        launch {
            while (true) {
                profiler.profileTo(MotionState(maxPos, 0.0))
                withTimeoutOrNull(1000) { runPid(maxPos) }
                profiler.profileTo(MotionState(minPos, 0.0))
                withTimeoutOrNull(1000) { runPid(minPos) }
            }
        }

        loopYieldWhile({ !gamepad1.y }) {
            telemetry["Current State"] = "Motion Profile"
            target = profiler.currentMotionState.x
        }

        freeState
    }

    private val pidState: FS = FS {
        G.cmd.launchCommand(subsystem.nel()) {
            runPid((minPos + maxPos) / 2)
        }

        suspendUntil { gamepad1.y }
        freeState
    }

    override suspend fun runSuspendOpMode() {
        suspendUntilStart()

        coroutineScope {
            launch { runStateMachine(freeState) }

            loopYieldWhile({ true }) {
                updateCoefs()

                telemetry["pos"] = readInput()
                telemetry["target"] = target
                telemetry["error"] = target - readInput()
            }
        }
    }
}