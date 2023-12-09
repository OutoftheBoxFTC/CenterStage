package org.firstinspires.ftc.teamcode.actions.hardware

import arrow.core.nel
import arrow.optics.optics
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator
import com.acmerobotics.roadrunner.profile.MotionState
import com.outoftheboxrobotics.suspendftc.loopYieldWhile
import com.outoftheboxrobotics.suspendftc.suspendUntil
import com.qualcomm.robotcore.util.ElapsedTime
import kotlinx.coroutines.cancelAndJoin
import org.firstinspires.ftc.teamcode.RobotState
import org.firstinspires.ftc.teamcode.actions.controllers.FeedforwardCoefs
import org.firstinspires.ftc.teamcode.actions.controllers.PidCoefs
import org.firstinspires.ftc.teamcode.actions.controllers.runMotionProfile
import org.firstinspires.ftc.teamcode.actions.controllers.runPidController
import org.firstinspires.ftc.teamcode.Subsystem
import org.firstinspires.ftc.teamcode.extensionState
import org.firstinspires.ftc.teamcode.mainLooper
import org.firstinspires.ftc.teamcode.util.G
import kotlin.math.abs

@optics
data class ExtensionState(
    val encoderBias: Int
) { companion object }

object ExtensionConfig {
    const val pidRange = 200

    val pidCoefs: PidCoefs = PidCoefs(0.04, 0.0, 0.0001)
}

fun extensionLength() = G.ehub.extension.currentPosition - G[RobotState.extensionState.encoderBias]
fun resetExtensionLength(newPos: Int = 0) {
    G[RobotState.extensionState.encoderBias] = extensionLength() - newPos
}

fun setExtensionPower(power: Double) { G.ehub.extension.power = power }
suspend fun runExtensionTo(
    target: Int,
    timeout: Long = 2000,
    tolerance: Int = 16,
    keepPid: Boolean = false
) {
    G.cmd.runNewCommand(Subsystem.EXTENSION.nel()) {
        loopYieldWhile({ abs(extensionLength() - target) < ExtensionConfig.pidRange }) {
            setExtensionPower(if (extensionLength() < target) 1.0 else -1.0)
        }
    }

    val pidJob = launchExtensionPid(target)
    val timer = ElapsedTime()

    suspendUntil {
        timer.milliseconds() > timeout || abs(extensionLength() - target) <= tolerance
    }

    if (!keepPid) pidJob.cancelAndJoin()
}

fun launchExtensionPid(target: Int) = G[RobotState.mainLooper].scheduleCoroutine {
    G.cmd.runNewCommand(Subsystem.EXTENSION.nel()) {
        runPidController(
            coefs = ExtensionConfig.pidCoefs,
            input = { extensionLength().toDouble() },
            target = { target.toDouble() },
            output = ::setExtensionPower
        )
    }
}
