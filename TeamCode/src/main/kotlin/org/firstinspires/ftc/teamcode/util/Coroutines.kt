package org.firstinspires.ftc.teamcode.util

import arrow.core.Nel
import arrow.fx.coroutines.Race3
import com.outoftheboxrobotics.suspendftc.loopYieldWhile
import com.outoftheboxrobotics.suspendftc.suspendUntil
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.launch
import org.firstinspires.ftc.teamcode.CommandHandler
import org.firstinspires.ftc.teamcode.Subsystem

/**
 * [loopYieldWhile] that repeats forever, so this function never returns.
 */
suspend inline fun mainLoop(block: () -> Unit): Nothing {
    loopYieldWhile({ true }, block)
    error("Return from mainLoop()")
}

suspend inline fun suspendUntilRisingEdge(predicate: () -> Boolean) {
    suspendUntil { !predicate() }
    suspendUntil(predicate)
}

suspend inline fun suspendUntilFallingEdge(predicate: () -> Boolean) {
    suspendUntil(predicate)
    suspendUntil { !predicate() }
}

/**
 * Launches a new command with the [CommandHandler] using the [CoroutineScope] context receiver.
 *
 * @param required The required subsystems for the command.
 * @param action The action to run in the command.
 */
context(CoroutineScope)
fun CommandHandler.launchCommand(required: Nel<Subsystem>, action: suspend () -> Unit) = launch {
    runNewCommand(required, action)
}

/**
 * Merges a [Race3] into a single value.
 */
fun <T> Race3<T, T, T>.merge() = fold({ it }, { it }, { it })
