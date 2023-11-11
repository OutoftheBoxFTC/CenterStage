package org.firstinspires.ftc.teamcode

import com.outoftheboxrobotics.suspendftc.loopYieldWhile
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.first
import kotlinx.coroutines.flow.withIndex
import kotlinx.coroutines.launch
import org.firstinspires.ftc.robotcore.external.Telemetry
import kotlin.properties.ReadOnlyProperty
import kotlin.properties.ReadWriteProperty
import kotlin.reflect.KProperty

typealias G = Globals
typealias C = Controls

interface ReadOnlyProperty<out V> : ReadOnlyProperty<Any?, V> {
    val value: V

    override fun getValue(thisRef: Any?, property: KProperty<*>) = value
}

interface ReadWriteProperty<V> : ReadWriteProperty<Any?, V> {
    var value: V

    override fun getValue(thisRef: Any?, property: KProperty<*>) = value
    override fun setValue(thisRef: Any?, property: KProperty<*>, value: V) { this.value = value }
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

operator fun Telemetry.set(caption: String, value: Any) { addData(caption, value) }
