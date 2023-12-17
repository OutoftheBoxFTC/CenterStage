package org.firstinspires.ftc.teamcode.opmodes.testing

import com.outoftheboxrobotics.suspendftc.loopYieldWhile
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.opmodes.RobotOpMode
import org.firstinspires.ftc.teamcode.util.G
import org.firstinspires.ftc.teamcode.util.set

@TeleOp
@Disabled
class IntakeTest : RobotOpMode(
    runMultiThreaded = true,
    imuRunMode = ImuRunMode.THREADED
) {
    override suspend fun runSuspendOpMode() {
        suspendUntilStart()

        val timer = ElapsedTime()
        var servoPos = 0.144

        val increment = 0.05

        loopYieldWhile({ true }) {
            val dt = timer.seconds()
            timer.reset()

            when {
                gamepad1.right_bumper -> servoPos += increment * dt
                gamepad1.left_bumper -> servoPos -= increment * dt
            }

            G.ehub.intakeRoller.power = -gamepad1.left_stick_y.toDouble()

            G.ehub.intakeTilt.position = servoPos
            telemetry["servo position"] = servoPos
        }
    }
}