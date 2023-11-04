package org.firstinspires.ftc.teamcode.logging

import java.util.concurrent.ConcurrentHashMap

class Sublog(private val collector: (Map<String, Any?>) -> Unit) {
    private val entries = ConcurrentHashMap<String, Any?>()
    private val subEntries = ConcurrentHashMap<String, Map<String, Any?>>()

    operator fun set(key: String, value: Any?) = entries.set(key, value)
    fun collect() {
        val subEntryMap = buildMap {
            subEntries.flatMap { (k, v) -> v.mapKeys { "$k.${it.key}" }.entries }.forEach {
                set(it.key, it.value)
            }
        }

        collector(entries + subEntryMap)

        entries.clear()
    }

    fun sublog(name: String) = Sublog {
        if (it.isNotEmpty()) subEntries[name] = it
        else subEntries.remove(name)
    }
}