package org.firstinspires.ftc.teamcode.opmodes.testing

import arrow.core.some
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.outoftheboxrobotics.suspendftc.suspendUntil
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.hardware.devices.ThreadedImuHandler
import org.firstinspires.ftc.teamcode.opmodes.RobotOpMode
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.util.G
import kotlin.math.PI

@Autonomous
class RoadrunnerIntegrationTest : RobotOpMode(
    runMultiThreaded = true,
    imuHandler = ThreadedImuHandler().some()
) {
    override suspend fun runSuspendOpMode() {
        val traj = SampleMecanumDrive.trajectorySequenceBuilder(Pose2d()).apply {
            setTangent(-PI/3)
            splineToSplineHeading(Pose2d(24.0, -18.0, PI / 2),-PI/2)
        }.build()

        suspendUntilStart()

        G.drive.followTrajectory(traj)
        G.drive.launchFixpoint(traj.end())

        suspendUntil { false }
    }
}