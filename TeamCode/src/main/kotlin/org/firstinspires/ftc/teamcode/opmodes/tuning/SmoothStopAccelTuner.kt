package org.firstinspires.ftc.teamcode.opmodes.tuning

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.outoftheboxrobotics.suspendftc.loopYieldWhile
import com.outoftheboxrobotics.suspendftc.suspendUntil
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.actions.hardware.currentDrivePose
import org.firstinspires.ftc.teamcode.actions.hardware.setDrivePowers
import org.firstinspires.ftc.teamcode.opmodes.RobotOpMode
import org.firstinspires.ftc.teamcode.util.set

@TeleOp
class SmoothStopAccelTuner : RobotOpMode() {
    override suspend fun runSuspendOpMode() {
        suspendUntilStart()

        var a = 0.0

        while (true) {
            loopYieldWhile({ !gamepad1.a }) {
                telemetry["Accel"] = a
            }

            setDrivePowers(Pose2d(1.0, 0.0, 0.0))

            val timer = ElapsedTime()

            suspendUntil { gamepad1.x }
            timer.reset()
            val startVec = currentDrivePose().vec()
            setDrivePowers(Pose2d())

            suspendUntil { !gamepad1.x }
            val t = timer.seconds()
            val x = (currentDrivePose().vec() - startVec).norm()

            a = 2*x / (t*t)
        }
    }
}