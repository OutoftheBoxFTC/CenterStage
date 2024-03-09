package org.firstinspires.ftc.teamcode.opmodes.tuning

import com.outoftheboxrobotics.suspendftc.loopYieldWhile
import com.outoftheboxrobotics.suspendftc.suspendUntil
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.actions.hardware.DriveConfig
import org.firstinspires.ftc.teamcode.actions.hardware.extensionLength
import org.firstinspires.ftc.teamcode.actions.hardware.setExtensionPower
import org.firstinspires.ftc.teamcode.opmodes.RobotOpMode
import org.firstinspires.ftc.teamcode.util.G
import org.firstinspires.ftc.teamcode.util.set

@TeleOp
class ExtensionAccelTuner : RobotOpMode() {
    override suspend fun runSuspendOpMode() {
        suspendUntilStart()

        var a = 0.0

        while (true) {
            loopYieldWhile({ !gamepad1.x }) {
                telemetry["Accel"] = a
            }

            setExtensionPower(1.0)

            val timer = ElapsedTime()

            suspendUntil { !gamepad1.x }
            timer.reset()
            val startPos = extensionLength()
            setExtensionPower(0.0)

            suspendUntil { G.ehub.extension.velocity <= 0 }

            val t = timer.seconds()
            val x = (extensionLength() - startPos) * DriveConfig.intakeOdoExtensionMultiplier

            a = 2*x / (t*t)
        }
    }
}