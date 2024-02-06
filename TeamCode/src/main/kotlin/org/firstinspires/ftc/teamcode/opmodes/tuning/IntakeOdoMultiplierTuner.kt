package org.firstinspires.ftc.teamcode.opmodes.tuning

import com.outoftheboxrobotics.suspendftc.loopYieldWhile
import com.outoftheboxrobotics.suspendftc.yieldLooper
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.actions.hardware.currentDrivePose
import org.firstinspires.ftc.teamcode.opmodes.RobotOpMode
import org.firstinspires.ftc.teamcode.util.G
import org.firstinspires.ftc.teamcode.util.mainLoop
import org.firstinspires.ftc.teamcode.util.set
import kotlin.math.PI

@TeleOp
class IntakeOdoMultiplierTuner : RobotOpMode() {
    override suspend fun runSuspendOpMode() {
        suspendUntilStart()

        val startPos = G.chub.odoIntake.currentPosition

        var lastPose = currentDrivePose()
        var distance = 0.0

        yieldLooper()

        loopYieldWhile({ !gamepad1.x }) {
            val poseDiff = currentDrivePose() - lastPose
            lastPose = currentDrivePose()

            distance += poseDiff.vec() dot lastPose.headingVec().rotated(PI / 2)

            telemetry["distance"] = distance
            telemetry["ticks"] = G.chub.odoIntake.currentPosition - startPos
        }

        mainLoop {
            telemetry["distance"] = distance
            telemetry["ticks"] = G.chub.odoIntake.currentPosition - startPos
        }
    }
}