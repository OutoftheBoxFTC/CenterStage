package org.firstinspires.ftc.teamcode.opmodes.testing

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import org.firstinspires.ftc.teamcode.actions.hardware.launchFixpoint
import org.firstinspires.ftc.teamcode.opmodes.RobotOpMode
import org.firstinspires.ftc.teamcode.util.mainLoop

@Autonomous
@Disabled
class FixpointTest : RobotOpMode() {
    override suspend fun runSuspendOpMode() {
        suspendUntilStart()

        launchFixpoint(Pose2d())

        mainLoop {  }
    }
}