package org.firstinspires.ftc.teamcode.opmodes.testing

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.outoftheboxrobotics.suspendftc.suspendUntil
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.actions.hardware.followTrajectoryFixpoint
import org.firstinspires.ftc.teamcode.actions.hardware.followTrajectoryPath
import org.firstinspires.ftc.teamcode.actions.hardware.setDrivetrainIdle
import org.firstinspires.ftc.teamcode.opmodes.RobotOpMode
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.util.suspendUntilRisingEdge
import kotlin.math.PI

@Autonomous
class TrajectoryFollowingTest : RobotOpMode() {
    override suspend fun runSuspendOpMode() {
        val traj = SampleMecanumDrive.trajectorySequenceBuilder(Pose2d()).apply {
            setTangent(-PI/3)
            splineToSplineHeading(Pose2d(24.0, -18.0, PI / 2),-PI/2)
        }.build()

        suspendUntilStart()

        while (true) {
            followTrajectoryFixpoint(traj)
            suspendUntilRisingEdge { gamepad1.x }
            setDrivetrainIdle()
            suspendUntilRisingEdge { gamepad1.x }
            followTrajectoryPath(traj)
            suspendUntil { gamepad1.x }
        }
    }
}