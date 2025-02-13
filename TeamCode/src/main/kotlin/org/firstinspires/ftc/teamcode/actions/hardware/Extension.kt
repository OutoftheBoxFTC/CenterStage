package org.firstinspires.ftc.teamcode.actions.hardware

import arrow.core.nel
import arrow.optics.optics
import com.outoftheboxrobotics.suspendftc.loopYieldWhile
import com.outoftheboxrobotics.suspendftc.suspendFor
import com.outoftheboxrobotics.suspendftc.suspendUntil
import com.qualcomm.robotcore.util.ElapsedTime
import com.qualcomm.robotcore.util.RobotLog
import kotlinx.coroutines.cancelAndJoin
import org.firstinspires.ftc.teamcode.RobotState
import org.firstinspires.ftc.teamcode.Subsystem
import org.firstinspires.ftc.teamcode.actions.controllers.FeedforwardCoefs
import org.firstinspires.ftc.teamcode.actions.controllers.PidCoefs
import org.firstinspires.ftc.teamcode.actions.controllers.runPidController
import org.firstinspires.ftc.teamcode.actions.controllers.runVeloPid
import org.firstinspires.ftc.teamcode.extensionState
import org.firstinspires.ftc.teamcode.mainLooper
import org.firstinspires.ftc.teamcode.opmodes.tuning.ExtensionVeloPidTuner
import org.firstinspires.ftc.teamcode.util.G
import kotlin.math.abs

/**
 * @param encoderBias The encoder value of the extension when it is fully retracted
 */
@optics
data class ExtensionState(
    val encoderBias: Int
) { companion object }

object ExtensionConfig {
    const val extensionHoldPower = -0.15

    val extensionVeloPid = PidCoefs(0.0001, 0.0015, 0.000005)
    val extensionFeedforward = FeedforwardCoefs(0.0, 0.003, 0.001, 0.00005)
}

fun extensionLength() = G.ehub.extension.currentPosition - G[RobotState.extensionState.encoderBias]
fun resetExtensionLength(newPos: Int = 0) {
    G[RobotState.extensionState.encoderBias] = G.ehub.extension.currentPosition - newPos
}

fun setExtensionPower(power: Double) { G.ehub.extension.power = power }

/**
 * Gives the extension a constant power to hold its retract position
 */
fun setExtensionHold() = setExtensionPower(ExtensionConfig.extensionHoldPower)

/**
 * Runs the extension to the given encoder position
 *
 * @param target The target encoder position
 * @param timeout The maximum time to run the extension
 * @param tolerance The maximum error from the target position to be considered "at" the target
 * @param keepPid Whether to keep the PID controller running after the extension reaches the target
 */
suspend fun runExtensionTo(
    target: Int,
    timeout: Long = 2000,
    tolerance: Int = 30,
    keepPid: Boolean = false,
    maxVel: Double =  1500.0,
    maxAccel: Double = 3000.0
) {
    val pidJob = launchExtensionPid(target, maxVel, maxAccel)
    val timer = ElapsedTime()

    suspendUntil {
        timer.milliseconds() > timeout || abs(extensionLength() - target) <= tolerance
    }

    if (!keepPid) {
        pidJob.cancelAndJoin()
        ezStopExtension()
    }
}

suspend fun ezExtend(
    target: Int,
    timeout: Long = 2000,
    stopDist: Int = 30,
    power: Double = 1.0
) {
    G.ehub.extension.power = power

    val timer = ElapsedTime()

    suspendUntil { target - extensionLength() < stopDist || timer.milliseconds() > timeout }

    if (timer.milliseconds() > timeout) {
        RobotLog.w("Timeout $timeout ms on ezExtend, target $target, actual ${extensionLength()}")
    }

    ezStopExtension()
}

/**
 * Retracts the extension until the limit switch is pressed, then resets the encoder position.
 */
suspend fun retractExtension() {
    setExtensionPower(-1.0)
    suspendUntil { G.chub.extensionLimitSwitch }
    resetExtensionLength()
    setExtensionPower(-0.15)
}

suspend fun ezStopExtension() {
    setExtensionPower(-1.0)
    suspendFor(20)
    setExtensionPower(0.0)
}

/**
 * Launches a Velocity PID controller to the given target encoder position.
 */
fun launchExtensionPid(
    target: Int,
    maxVel: Double = 1500.0, maxAccel: Double = 3000.0
) = G[RobotState.mainLooper].scheduleCoroutine {
    G.cmd.runNewCommand(Subsystem.EXTENSION.nel()) {
        runVeloPid(
            feedforward = ExtensionConfig.extensionFeedforward,
            pid = ExtensionConfig.extensionVeloPid,
            input = { extensionLength().toDouble() },
            target = { target.toDouble() },
            output = { G.ehub.extension.power = it },
            maxVel = maxVel,
            maxAccel = maxAccel,
            hz = 30
        )
    }
}
