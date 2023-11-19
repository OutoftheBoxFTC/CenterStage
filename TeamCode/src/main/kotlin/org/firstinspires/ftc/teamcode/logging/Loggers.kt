package org.firstinspires.ftc.teamcode.logging

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import kotlinx.coroutines.channels.Channel
import kotlinx.coroutines.channels.getOrElse
import org.firstinspires.ftc.robotcore.external.Telemetry

@Config
class Loggers(telemetry: Telemetry) {
    companion object {
        @JvmField
        var queryString = ""
    }

    val packetChannel = Channel<TelemetryPacket>(capacity = Channel.UNLIMITED)

    private var lastPacket: TelemetryPacket? = null

    val rootLog = Sublog { entries ->
        val filtered = if (queryString == "*") entries else {
            val allowedKeys = queryString.split(';').filter { it.isNotBlank() }

            entries.filterKeys { key ->
                allowedKeys.any { key.startsWith(it) }
            }
        }

        filtered.forEach { (k, v) ->
            telemetry.addData(k, v)
        }

        lastPacket = packetChannel.tryReceive().getOrElse { lastPacket }

        lastPacket?.let {
            FtcDashboard.getInstance().sendTelemetryPacket(it)
        }

        telemetry.update()
    }

    val imu = rootLog.sublog("imu")
    val drive = rootLog.sublog("drive")
    val hardware = rootLog.sublog("hardware")
    val vision = rootLog.sublog("vision")
}