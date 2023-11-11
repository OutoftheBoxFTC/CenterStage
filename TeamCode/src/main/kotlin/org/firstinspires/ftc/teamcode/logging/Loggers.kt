package org.firstinspires.ftc.teamcode.logging

import com.acmerobotics.dashboard.config.Config
import org.firstinspires.ftc.robotcore.external.Telemetry

@Config
class Loggers(telemetry: Telemetry) {
    companion object {
        @JvmField
        var queryString = ""
    }

    val rootLog = Sublog { entries ->
        val filtered = if (queryString == "*") entries else {
            val allowedKeys = queryString.split(';')

            entries.filterKeys { key -> allowedKeys.any { it.startsWith(key) } }
        }

        filtered.forEach { (k, v) ->
            telemetry.addData(k, v)
        }

        telemetry.update()
    }

    val imu = rootLog.sublog("imu")
    val drive = rootLog.sublog("drive")
    val hardware = rootLog.sublog("hardware")
    val vision = rootLog.sublog("vision")
}