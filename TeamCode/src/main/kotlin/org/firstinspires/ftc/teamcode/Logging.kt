package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.config.Config
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit

private interface TreeLog {
    fun put(group: String, key: String, value: Any?)

    operator fun String.invoke(block: TreeLog.() -> Unit) = object : TreeLog {
        override fun put(group: String, key: String, value: Any?) =
            this@TreeLog.put(
                this@invoke + if (group.isNotBlank()) ".$group" else "",
                key,
                value
            )
    }.run(block)

    infix fun String.set(value: Any?) = put("", this, value)
}

private fun TreeLog.rootLog(state: RobotState) {
    "imu" {
        "IMU Angle" set state.imuState.angle

        "internal" {
            "Raw IMU Angle" set state.imuState.rawAngle
            "IMU Angle Bias" set state.imuState.angleBias
            "Using Threaded IMU" set (state.imuState.threadedImuJob != null)
        }
    }

    "localizer" {
        val pose = state.driveState.currentPose

        "x" set pose.x
        "y" set pose.y
        "heading" set pose.heading
    }

    "drive" {
        "Drive State" set state.driveState.driveControlState
    }

    "current" {
        val chub = state.chub
        val ehub = state.ehub

        "chub" {
            val currents = listOf(
                chub.tl.getCurrent(CurrentUnit.AMPS),
                chub.tr.getCurrent(CurrentUnit.AMPS),
                chub.bl.getCurrent(CurrentUnit.AMPS),
                chub.br.getCurrent(CurrentUnit.AMPS)
            )

            "Drive tl current" set currents[0]
            "Drive tr current" set currents[1]
            "Drive bl current" set currents[2]
            "Drive br current" set currents[3]

            "Drive total current" set currents.sum()
        }

        "ehub" {
            "Extension current" set ehub.extension.getCurrent(CurrentUnit.AMPS)
            "Intake roller current" set ehub.intakeRoller.getCurrent(CurrentUnit.AMPS)
            "Lift current" set ehub.outtakeLift.getCurrent(CurrentUnit.AMPS)
        }
    }
}

@Config
object LoggingConfig {
    @JvmField var queryString = ""
}

fun Telemetry.logState(state: RobotState) = object : TreeLog {
    override fun put(group: String, key: String, value: Any?) {
        val str = LoggingConfig.queryString

        if (
            str == "*" ||
            str
                .split(';')
                .filter { it.isNotBlank() }
                .any { group.startsWith(it) }
        ) addData(key, value)
    }
}.rootLog(state)
