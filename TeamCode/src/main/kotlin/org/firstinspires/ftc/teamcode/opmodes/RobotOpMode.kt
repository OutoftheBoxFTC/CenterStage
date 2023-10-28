package org.firstinspires.ftc.teamcode.opmodes

import arrow.fx.coroutines.autoCloseable
import arrow.fx.coroutines.resourceScope
import arrow.optics.copy
import com.outoftheboxrobotics.suspendftc.Looper
import com.outoftheboxrobotics.suspendftc.suspendUntil
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import kotlinx.coroutines.DelicateCoroutinesApi
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.ExperimentalCoroutinesApi
import kotlinx.coroutines.launch
import kotlinx.coroutines.newSingleThreadContext
import kotlinx.coroutines.runBlocking
import kotlinx.coroutines.withContext
import org.firstinspires.ftc.teamcode.Globals
import org.firstinspires.ftc.teamcode.Globals.defaultRobotState
import org.firstinspires.ftc.teamcode.RobotState
import org.firstinspires.ftc.teamcode.driveLooper
import org.firstinspires.ftc.teamcode.looper
import org.firstinspires.ftc.teamcode.mainLooper

abstract class RobotOpMode(
    private val runMultiThreaded: Boolean = false,
    private val monitorOpmodeStop: Boolean = true
) : LinearOpMode() {
    abstract suspend fun runSuspendOpMode()

    protected suspend fun suspendUntilStart() {
        withContext(Dispatchers.IO) {
            waitForStart()
        }
    }

    @OptIn(ExperimentalCoroutinesApi::class, DelicateCoroutinesApi::class)
    override fun runOpMode() {
        val mainLooperLens = RobotState.looper.mainLooper
        val driveLooperLens = RobotState.looper.driveLooper

        defaultRobotState(hardwareMap).copy {
            if (runMultiThreaded) {
                driveLooperLens set Looper()
            }
        }.let { Globals.initializeRobotState(it) }

        (Globals[driveLooperLens] ?: Globals[mainLooperLens])
            .scheduleCoroutine {
                Globals.chub.syncHardware()
            }

        Globals[mainLooperLens].scheduleCoroutine {
            Globals.ehub.syncHardware()
        }

        Globals[mainLooperLens].scheduleCoroutine {
            val job = launch { runSuspendOpMode() }

            suspendUntil { job.isCompleted || (monitorOpmodeStop && !opModeIsActive()) }

            Globals.stop()
        }

        val driveThread = autoCloseable { newSingleThreadContext("Drive thread") }

        runBlocking {
            resourceScope {
                Globals[driveLooperLens]?.let {
                    launch(driveThread.bind()) {
                        it.run()
                    }
                }

                Globals[mainLooperLens].run()
            }
        }
    }
}