package org.firstinspires.ftc.teamcode.logging

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.Globals
import org.firstinspires.ftc.teamcode.RobotState
import org.firstinspires.ftc.teamcode.actions.hardware.DriveControlState
import org.firstinspires.ftc.teamcode.driveState

@Config
class Loggers(telemetry: Telemetry) {
    companion object {
        @JvmField
        var queryString = ""
    }

    val rootLog = Sublog { entries ->
        val filtered = if (queryString == "*") entries else {
            val allowedKeys = queryString.split(';').filter { it.isNotBlank() }

            entries.filterKeys { key ->
                allowedKeys.any { key.startsWith(it) }
            }
        }

        val driveState = Globals[RobotState.driveState]

        driveState.rrDrive?.let {
            val packet = it.getDriveTelemetryPacket(
                (driveState.driveControlState as? DriveControlState.Fixpoint)?.target
            )

            FtcDashboard.getInstance().sendTelemetryPacket(packet)
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