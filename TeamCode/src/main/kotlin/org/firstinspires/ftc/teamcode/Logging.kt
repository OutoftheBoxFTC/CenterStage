package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.config.Config
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import org.firstinspires.ftc.teamcode.actions.hardware.extensionLength
import org.firstinspires.ftc.teamcode.actions.hardware.liftPos

/**
 * Convenience DSL for logging data to telemetry.
 *
 * "Subtrees" pipe telemetry data upstream and append their own group name.
 *
 * Example:
 * ```
 * "group_name" {
 *   "key" set value    // group_name
 *   "subgroup" {
 *     "key" set value  // group_name.subgroup
 *   }
 * }
 * ```
 */
private interface TreeLog {
    fun put(group: String, key: String, value: Any?)

    operator fun String.invoke(block: TreeLog.() -> Unit) = object : TreeLog {
        override fun put(group: String, key: String, value: Any?) =
            // Pipe data upstream with group name appended at the beginning
            this@TreeLog.put(
                this@invoke + if (group.isNotBlank()) ".$group" else "",
                key,
                value
            )
    }.run(block)

    infix fun String.set(value: Any?) = put("", this, value)
}

/**
 * Logs the robot state to telemetry.
 */
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

    "extension" {
        "Extension length" set extensionLength()
        "Extension limit switch" set state.chub.extensionLimitSwitch
    }

    "lift" {
        "Lift pos" set liftPos()
    }

    "looptime" {
        "Chub loop frequency (Hz)" set state.chub.lastLoopFrequency
        "Ehub loop frequency (Hz)" set state.ehub.lastLoopFrequency
    }

    "power" {
        val chub = state.chub
        val ehub = state.ehub

        "voltage" {
            "Voltage" set chub.voltageSensor.voltage
        }

        "Total Current" set chub.hubCurrent + ehub.hubCurrent

        "chub" {
            "Chub current" set chub.hubCurrent

            val driveCurrents = listOf(
                chub.tl.getCurrent(CurrentUnit.AMPS),
                chub.tr.getCurrent(CurrentUnit.AMPS),
                chub.bl.getCurrent(CurrentUnit.AMPS),
                chub.br.getCurrent(CurrentUnit.AMPS)
            )

            "Drive tl current" set driveCurrents[0]
            "Drive tr current" set driveCurrents[1]
            "Drive bl current" set driveCurrents[2]
            "Drive br current" set driveCurrents[3]

            "Drive total current" set driveCurrents.sum()
        }

        "ehub" {
            "Ehub current" set ehub.hubCurrent

            "Extension current" set ehub.extension.getCurrent(CurrentUnit.AMPS)
            "Intake roller current" set ehub.intakeRoller.getCurrent(CurrentUnit.AMPS)
            "Lift current" set ehub.outtakeLift.getCurrent(CurrentUnit.AMPS)
        }
    }
}

/**
 * FTCDashboard config for selecting which data to show in telemetry.
 *
 * Semicolons separate group names, items are filtered according to whether their group starts with
 * any of the group names. An asterisk matches all groups.
 */
@Config
object LoggingConfig {
    @JvmField var queryString = "localizer;extension;looptime;power.voltage"
}

/**
 * Logs the robot state to telemetry with filtering.
 */
fun Telemetry.logState(state: RobotState) = object : TreeLog {
    override fun put(group: String, key: String, value: Any?) {
        // Implementation of TreeLog for adding filtered data to telemetry
        val str = LoggingConfig.queryString

        if (
            str == "*" ||
            str
                .split(';')
                .filter { it.isNotBlank() }
                // Check if group name matches
                .any { group.startsWith(it) }
        ) addData(key, value)
    }
}.rootLog(state)
