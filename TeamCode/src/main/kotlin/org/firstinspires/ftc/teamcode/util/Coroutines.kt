package org.firstinspires.ftc.teamcode.util

import arrow.core.Nel
import arrow.fx.coroutines.Race3
import arrow.optics.Copy
import arrow.optics.copy
import com.outoftheboxrobotics.suspendftc.loopYieldWhile
import com.outoftheboxrobotics.suspendftc.suspendUntil
import com.outoftheboxrobotics.suspendftc.yieldLooper
import com.qualcomm.robotcore.util.ElapsedTime
import com.qualcomm.robotcore.util.MovingStatistics
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Job
import kotlinx.coroutines.cancelAndJoin
import kotlinx.coroutines.coroutineScope
import kotlinx.coroutines.flow.FlowCollector
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.distinctUntilChanged
import kotlinx.coroutines.flow.first
import kotlinx.coroutines.flow.map
import kotlinx.coroutines.flow.stateIn
import kotlinx.coroutines.flow.update
import kotlinx.coroutines.flow.withIndex
import kotlinx.coroutines.launch
import org.firstinspires.ftc.teamcode.CommandHandler
import org.firstinspires.ftc.teamcode.Subsystem
import kotlin.math.abs

/**
 * [loopYieldWhile] that repeats forever, so this function never returns. Can also set a fixed
 * frequency to run at.
 */
suspend inline fun mainLoop(hz: Int? = null, block: () -> Unit): Nothing {
    val targetDelay = hz?.let { 1.0 / it }

    val mainTimer = ElapsedTime()
    val cycleTimer = ElapsedTime()

    val dt = MovingStatistics(10)

    block()
    yieldLooper()

    while (true) {
        if (targetDelay == null) {
            block()
            yieldLooper()
            continue
        }

        dt.add(cycleTimer.seconds())
        cycleTimer.reset()

        val t = mainTimer.seconds()

        if (abs(t - targetDelay) < abs(t + dt.mean - targetDelay)) {
            block()
            mainTimer.reset()
        }

        yieldLooper()
    }
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

/**
 * Runs the given block and cancels the job when it finishes.
 */
suspend inline fun <T> Job.use(block: () -> T) = block().also { cancelAndJoin() }

/**
 * Creates a [StateFlow] that maps the values of this [StateFlow] using the given [transform] function.
 */
fun <U, V> StateFlow<U>.mapState(transform: (U) -> V) = object : StateFlow<V> {
    private val flow = this@mapState.map(transform)

    override val replayCache: List<V>
        get() = listOf(value)
    override val value: V
        get() = transform(this@mapState.value)

    override suspend fun collect(collector: FlowCollector<V>): Nothing = coroutineScope {
        flow.distinctUntilChanged().stateIn(this).collect(collector)
    }
}

/**
 * Atomically updates the current value of this [MutableStateFlow] using the arrow-optics [Copy] DSL.
 */
fun <T> MutableStateFlow<T>.modify(block: Copy<T>.(T) -> Unit) = update { it.copy { block(it) } }

/**
 * Suspends until a new value of this [StateFlow] is emitted.
 */
suspend fun <T> StateFlow<T>.next() =
    // We ignore the first value because StateFlow repeats the existing value.
    withIndex().first { it.index > 0 }.value