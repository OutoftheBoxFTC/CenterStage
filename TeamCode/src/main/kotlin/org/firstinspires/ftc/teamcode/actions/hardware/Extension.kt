package org.firstinspires.ftc.teamcode.actions.hardware

import arrow.optics.optics
import com.outoftheboxrobotics.suspendftc.loopYieldWhile
import com.outoftheboxrobotics.suspendftc.suspendUntil
import com.outoftheboxrobotics.tickt.withTicket
import com.qualcomm.robotcore.util.ElapsedTime
import kotlinx.coroutines.cancelAndJoin
import org.firstinspires.ftc.teamcode.RobotState
import org.firstinspires.ftc.teamcode.Subsystem
import org.firstinspires.ftc.teamcode.actions.controllers.PidCoefs
import org.firstinspires.ftc.teamcode.actions.controllers.runPidController
import org.firstinspires.ftc.teamcode.extensionState
import org.firstinspires.ftc.teamcode.mainLooper
import org.firstinspires.ftc.teamcode.util.G
import kotlin.coroutines.CoroutineContext
import kotlin.coroutines.EmptyCoroutineContext
import kotlin.math.abs

/**
 * @param encoderBias The encoder value of the extension when it is fully retracted
 */
@optics
data class ExtensionState(
    val encoderBias: Int
) { companion object }

object ExtensionConfig {
    const val pidRange = 200

    val pidCoefs: PidCoefs = PidCoefs(0.04, 0.0, 0.0001)
    const val extensionHoldPower = -0.15
}

fun extensionLength() = G.ehub.extension.currentPosition - G[RobotState.extensionState.encoderBias]
fun resetExtensionLength(newPos: Int = 0) {
    G[RobotState.extensionState.encoderBias] = extensionLength() - newPos
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
    keepPid: Boolean = false
) = withTicket(Subsystem.EXTENSION) {
    loopYieldWhile({ abs(extensionLength() - target) < ExtensionConfig.pidRange }) {
        setExtensionPower(if (extensionLength() < target) 1.0 else -1.0)
    }

    val pidJob = launchExtensionPid(target, context = coroutineContext)
    val timer = ElapsedTime()

    suspendUntil {
        timer.milliseconds() > timeout || abs(extensionLength() - target) <= tolerance
    }

    if (!keepPid) pidJob.cancelAndJoin()
}

/**
 * Retracts the extension until the limit switch is pressed, then resets the encoder position.
 */
suspend fun retractExtension() = withTicket(Subsystem.EXTENSION) {
    setExtensionPower(-1.0)
    suspendUntil { G.chub.extensionLimitSwitch }
    resetExtensionLength()
    setExtensionPower(-0.15)
}

/**
 * Launches a PID controller to the given target encoder position.
 *
 * For ticketing to work properly, the coroutine context must be specified.
 */
fun launchExtensionPid(target: Int, context: CoroutineContext = EmptyCoroutineContext) =
    G[RobotState.mainLooper].scheduleCoroutine(G.scheduler + context) {
        withTicket(Subsystem.EXTENSION) {
            runPidController(
                coefs = ExtensionConfig.pidCoefs,
                input = { extensionLength().toDouble() },
                target = { target.toDouble() },
                output = ::setExtensionPower
            )
        }
    }
