package org.firstinspires.ftc.teamcode.opmodes.testing

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.outoftheboxrobotics.suspendftc.suspendUntil
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.actions.hardware.followLinePath
import org.firstinspires.ftc.teamcode.actions.hardware.launchFixpoint
import org.firstinspires.ftc.teamcode.actions.hardware.lineTo
import org.firstinspires.ftc.teamcode.opmodes.RobotOpMode

@TeleOp
class LineFollowerTest : RobotOpMode() {
    override suspend fun runSuspendOpMode() {
        val targetPose = Pose2d(96.0, 0.0, 0.0)

        suspendUntilStart()

        while (true) {
            suspendUntil { gamepad1.y }
            followLinePath(Vector2d(), targetPose.vec(), targetPose.heading)
            launchFixpoint(targetPose)
            suspendUntil { gamepad1.y }
            lineTo(Pose2d())
        }
    }
}