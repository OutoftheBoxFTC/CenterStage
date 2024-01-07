package org.firstinspires.ftc.teamcode.opmodes.tuning

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.util.MovingStatistics
import org.firstinspires.ftc.teamcode.RobotState
import org.firstinspires.ftc.teamcode.actions.hardware.DriveConfig
import org.firstinspires.ftc.teamcode.actions.hardware.retractExtension
import org.firstinspires.ftc.teamcode.actions.hardware.rrDrive
import org.firstinspires.ftc.teamcode.actions.hardware.setDrivePowers
import org.firstinspires.ftc.teamcode.driveState
import org.firstinspires.ftc.teamcode.opmodes.RobotOpMode
import org.firstinspires.ftc.teamcode.util.C
import org.firstinspires.ftc.teamcode.util.G
import org.firstinspires.ftc.teamcode.util.mainLoop
import org.firstinspires.ftc.teamcode.util.set
import kotlin.math.PI
import kotlin.math.abs

@TeleOp
class IntakeOdoDistanceTuner : RobotOpMode() {
    override suspend fun runSuspendOpMode() {
        suspendUntilStart()

        retractExtension()

        var lastRad = 0.0
        val radStats = MovingStatistics(200)

        mainLoop {
            val poseVel = G[RobotState.driveState.rrDrive]!!.poseVelocity!!
            val wheelVel = G.chub.odoIntake.correctedVelocity * DriveConfig.intakeOdoMultiplier

            if (abs(poseVel.heading) > PI / 3) {
                lastRad = (wheelVel + poseVel.y) / poseVel.heading
                radStats.add(lastRad)
            }

            telemetry["rad"] = lastRad
            telemetry["avg rad"] = radStats.mean

            setDrivePowers(0.0, 0.0, C.driveTurn)
        }
    }
}