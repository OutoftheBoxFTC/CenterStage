package org.firstinspires.ftc.teamcode

import kotlin.properties.ReadOnlyProperty
import kotlin.properties.ReadWriteProperty
import kotlin.reflect.KProperty

interface ReadOnlyProperty<out V> : ReadOnlyProperty<Any?, V> {
    val value: V

    override fun getValue(thisRef: Any?, property: KProperty<*>) = value
}

interface ReadWriteProperty<V> : ReadWriteProperty<Any?, V> {
    var value: V

    override fun getValue(thisRef: Any?, property: KProperty<*>) = value
    override fun setValue(thisRef: Any?, property: KProperty<*>, value: V) { this.value = value }
}
