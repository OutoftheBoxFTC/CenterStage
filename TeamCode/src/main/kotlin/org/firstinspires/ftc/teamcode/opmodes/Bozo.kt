package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.LoggingConfig
import org.firstinspires.ftc.teamcode.RobotState
import org.firstinspires.ftc.teamcode.actions.hardware.currentDrivePose
import org.firstinspires.ftc.teamcode.actions.hardware.extendoPose
import org.firstinspires.ftc.teamcode.driveState
import org.firstinspires.ftc.teamcode.util.G
import org.firstinspires.ftc.teamcode.util.mainLoop
import org.firstinspires.ftc.teamcode.util.set

@Autonomous
class Bozo : RobotOpMode() {
    override suspend fun runSuspendOpMode() {
        val oldLogState = LoggingConfig.queryString

        try {
            LoggingConfig.queryString = "*"
            mainLoop {
                val cp = currentDrivePose()
                val ep = G[RobotState.driveState.extendoPose]

                telemetry["Drive Pose"] = "Pose2d(${cp.x}, ${cp.y}, ${cp.heading})"
                telemetry["Intake Pose"] = "Pose2d(${ep.x}, ${ep.y}, ${ep.heading})"
            }
        } finally {
            LoggingConfig.queryString = oldLogState
        }
    }
}