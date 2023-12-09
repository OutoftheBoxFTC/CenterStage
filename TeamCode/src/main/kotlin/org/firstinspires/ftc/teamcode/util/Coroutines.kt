package org.firstinspires.ftc.teamcode.util

import arrow.core.Nel
import arrow.fx.coroutines.Race3
import com.outoftheboxrobotics.suspendftc.loopYieldWhile
import com.outoftheboxrobotics.suspendftc.suspendUntil
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.channels.SendChannel
import kotlinx.coroutines.launch
import org.firstinspires.ftc.teamcode.CommandHandler
import org.firstinspires.ftc.teamcode.Subsystem

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

fun <T> SendChannel<T>.trySendJava(item: T) = trySend(item)

context(CoroutineScope)
fun CommandHandler.launchCommand(required: Nel<Subsystem>, action: suspend () -> Unit) = launch {
    runNewCommand(required, action)
}

fun <T> Race3<T, T, T>.merge() = fold({ it }, { it }, { it })
