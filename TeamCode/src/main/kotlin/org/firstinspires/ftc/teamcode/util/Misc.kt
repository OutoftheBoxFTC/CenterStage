@file:Suppress("unused")

package org.firstinspires.ftc.teamcode.util

import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.Config
import org.firstinspires.ftc.teamcode.Controls
import org.firstinspires.ftc.teamcode.FunctionalState
import org.firstinspires.ftc.teamcode.Globals
import kotlin.properties.ReadOnlyProperty
import kotlin.properties.ReadWriteProperty
import kotlin.reflect.KProperty

// Convenience type aliases
typealias G = Globals
typealias C = Controls
typealias FS = FunctionalState

// Interfaces that simplify property delegation
interface ReadOnlyProperty<out V> : ReadOnlyProperty<Any?, V> {
    val value: V

    override fun getValue(thisRef: Any?, property: KProperty<*>) = value
}

interface ReadWriteProperty<V> : ReadWriteProperty<Any?, V> {
    var value: V

    override fun getValue(thisRef: Any?, property: KProperty<*>) = value
    override fun setValue(thisRef: Any?, property: KProperty<*>, value: V) { this.value = value }
}

// Operator overload for adding to telemetry
operator fun Telemetry.set(caption: String, value: Any) { addData(caption, value) }

/**
 * Throws an error if [Config.FAIL_FAST] is true, otherwise returns the result of [fallback].
 */
inline fun <T> fail(message: String, fallback: () -> T) =
    if (Config.FAIL_FAST) error(message) else fallback()