package org.firstinspires.ftc.teamcode.opmodes.testing

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.outoftheboxrobotics.suspendftc.suspendFor
import com.outoftheboxrobotics.suspendftc.suspendUntil
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.RobotState
import org.firstinspires.ftc.teamcode.actions.hardware.currentDrivePose
import org.firstinspires.ftc.teamcode.actions.hardware.followTrajectoryPath
import org.firstinspires.ftc.teamcode.actions.hardware.rrDrive
import org.firstinspires.ftc.teamcode.actions.hardware.setDrivePowers
import org.firstinspires.ftc.teamcode.actions.hardware.setDrivetrainIdle
import org.firstinspires.ftc.teamcode.driveState
import org.firstinspires.ftc.teamcode.opmodes.RobotOpMode
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.util.G
import org.firstinspires.ftc.teamcode.util.suspendUntilRisingEdge
import kotlin.math.PI

@TeleOp
class TrajectoryFollowingTest : RobotOpMode() {
    override suspend fun runSuspendOpMode() {
        val traj = SampleMecanumDrive.trajectorySequenceBuilder(Pose2d()).apply {
            lineToSplineHeading(Pose2d(-60.0, 0.0, 0.0))
            splineToConstantHeading(Vector2d(-72.0, 24.0), PI / 2)
        }.build()

        suspendUntilStart()

        G.ehub.extension.power = -0.15

        while (true) {
//            suspendUntilRisingEdge { gamepad1.x }
//            followTrajectoryFixpoint(traj)
            suspendUntilRisingEdge { gamepad1.x }
            setDrivetrainIdle()
            setDrivePowers(0.0, 0.0, 0.0)
//            followTrajectoryFixpoint(traj2)
//            suspendUntilRisingEdge { gamepad1.x }
//            setDrivetrainIdle()
            suspendUntilRisingEdge { gamepad1.x }
            followTrajectoryPath(traj)

            val timer = ElapsedTime()

            suspendUntil { currentDrivePose().vec().distTo(traj.end().vec()) <= 1.0 || timer.seconds() > 0.5 }

            setDrivetrainIdle()

            setDrivePowers(
                Pose2d(
                    G[RobotState.driveState.rrDrive]!!.poseVelocity!!.vec().let { -it / it.norm() },
                    0.0
                )
            )

            suspendFor(50)

            setDrivePowers(0.0, 0.0, 0.0)
        }
    }
}