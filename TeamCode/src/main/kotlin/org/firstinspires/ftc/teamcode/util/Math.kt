package org.firstinspires.ftc.teamcode.util

import com.acmerobotics.roadrunner.geometry.Vector2d

infix fun Vector2d.cross(other: Vector2d) = x * other.y - y * other.x

val Double.deg get() = Math.toRadians(this)
val Int.deg get() = toDouble().deg

class Interplut(private val map: Map<Double, Double> = emptyMap()) : Map<Double, Double> by map {
    override fun get(key: Double) = map[key] ?: map.keys.sorted().let { keys ->
        keys.lastOrNull { it < key } to keys.firstOrNull { it > key }
    }.let { (a, b) ->
        if (a != null && b != null) lerp(map[a]!!, map[b]!!, ilerp(a, b, key))
        else null
    }

    val range = map.keys.let { it.min()..it.max() }
}

fun lerp(a: Double, b: Double, t: Double) = (1 - t) * a + t * b
fun ilerp(a: Double, b: Double, c: Double) = (c - a) / (b - a)
