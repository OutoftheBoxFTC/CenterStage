package org.firstinspires.ftc.teamcode.util

import kotlinx.coroutines.coroutineScope
import kotlinx.coroutines.flow.FlowCollector
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.distinctUntilChanged
import kotlinx.coroutines.flow.map
import kotlinx.coroutines.flow.stateIn

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
