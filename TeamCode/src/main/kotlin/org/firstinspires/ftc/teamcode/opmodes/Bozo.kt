package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import kotlinx.coroutines.coroutineScope
import kotlinx.coroutines.launch
import org.firstinspires.ftc.teamcode.LoggingConfig
import org.firstinspires.ftc.teamcode.RobotState
import org.firstinspires.ftc.teamcode.actions.hardware.currentDrivePose
import org.firstinspires.ftc.teamcode.actions.hardware.currentImuAngle
import org.firstinspires.ftc.teamcode.actions.hardware.extendoPose
import org.firstinspires.ftc.teamcode.actions.hardware.nextImuAngle
import org.firstinspires.ftc.teamcode.driveState
import org.firstinspires.ftc.teamcode.util.G
import org.firstinspires.ftc.teamcode.util.mainLoop
import org.firstinspires.ftc.teamcode.util.set
import kotlin.math.PI

@Autonomous
class Bozo : RobotOpMode() {
    override suspend fun runSuspendOpMode() {
        val oldLogState = LoggingConfig.queryString

        try {
            LoggingConfig.queryString = "*"
            coroutineScope {
                launch {
                    mainLoop {
                        G.imuStartingHeading = currentImuAngle() + PI
                    }
                }

                mainLoop {
                    val cp = currentDrivePose()
                    val ep = G[RobotState.driveState.extendoPose]

                    telemetry["Drive Pose"] =
                        String.format("Pose2d(%.3f, %.3f, %.3f)", cp.x, cp.y, cp.heading)
                    telemetry["Intake Pose"] =
                        String.format("Pose2d(%.3f, %.3f, %.3f)", ep.x, ep.y, ep.heading)

                    G.ehub.extension.power = -gamepad1.left_stick_y.toDouble()
                }
            }
        } finally {
            LoggingConfig.queryString = oldLogState
        }
    }
}