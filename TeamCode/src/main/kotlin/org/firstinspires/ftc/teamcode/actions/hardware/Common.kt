package org.firstinspires.ftc.teamcode.actions.hardware

import com.acmerobotics.roadrunner.geometry.Pose2d
import org.firstinspires.ftc.teamcode.RobotState
import org.firstinspires.ftc.teamcode.driveState
import org.firstinspires.ftc.teamcode.util.G

fun resetDrivePose(newPose: Pose2d = Pose2d()) {
    resetImuAngle(newPose.heading)
    G[RobotState.driveState.currentPose] = newPose
}
