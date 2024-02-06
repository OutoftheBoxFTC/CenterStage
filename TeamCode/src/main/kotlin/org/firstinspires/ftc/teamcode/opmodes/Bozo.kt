package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.LoggingConfig
import org.firstinspires.ftc.teamcode.util.mainLoop

@Autonomous
class Bozo : RobotOpMode() {
    override suspend fun runSuspendOpMode() {
        val oldLogState = LoggingConfig.queryString

        try {
            LoggingConfig.queryString = "*"
            mainLoop {  }
        } finally {
            LoggingConfig.queryString = oldLogState
        }
    }
}