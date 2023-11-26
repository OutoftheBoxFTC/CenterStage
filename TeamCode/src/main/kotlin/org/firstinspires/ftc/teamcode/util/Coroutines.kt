package org.firstinspires.ftc.teamcode.util

import arrow.core.Nel
import com.outoftheboxrobotics.suspendftc.loopYieldWhile
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.channels.SendChannel
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.first
import kotlinx.coroutines.flow.withIndex
import kotlinx.coroutines.launch
import org.firstinspires.ftc.teamcode.command.CommandHandler
import org.firstinspires.ftc.teamcode.command.Subsystem
import kotlin.properties.ReadOnlyProperty

suspend inline fun mainLoop(block: () -> Unit): Nothing {
    loopYieldWhile({ true }, block)
    error("Return from mainLoop()")
}

suspend fun <T> StateFlow<T>.next() = withIndex().first { it.index > 0 }.value

fun CoroutineScope.risingEdgeMonitor(observed: () -> Boolean): ReadOnlyProperty<Any?, Boolean> {
    var last = observed()
    var current = observed()

    launch {
        loopYieldWhile({ true }) {
            last = current
            current = observed()
        }
    }

    return ReadOnlyProperty { _, _ -> current && !last}
}

fun CoroutineScope.fallingEdgeMonitor(observed: () -> Boolean): ReadOnlyProperty<Any?, Boolean> {
    var last = observed()
    var current = observed()

    launch {
        loopYieldWhile({ true }) {
            last = current
            current = observed()
        }
    }

    return ReadOnlyProperty { _, _ -> !current && last}
}

fun <T> SendChannel<T>.trySendJava(item: T) = trySend(item)

context(CoroutineScope)
fun CommandHandler.launchCommand(required: Nel<Subsystem>, action: suspend () -> Unit) = launch {
    runNewCommand(required, action)
}