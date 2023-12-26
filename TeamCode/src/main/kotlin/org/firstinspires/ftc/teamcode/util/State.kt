package org.firstinspires.ftc.teamcode.util

import arrow.optics.Copy
import arrow.optics.copy
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
