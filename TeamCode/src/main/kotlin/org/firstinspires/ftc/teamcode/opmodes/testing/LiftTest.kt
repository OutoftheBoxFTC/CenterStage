package org.firstinspires.ftc.teamcode.opmodes.testing

import com.acmerobotics.dashboard.config.Config
import com.outoftheboxrobotics.suspendftc.yieldLooper
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import org.firstinspires.ftc.teamcode.opmodes.RobotOpMode
import org.firstinspires.ftc.teamcode.util.G
import org.firstinspires.ftc.teamcode.util.mainLoop
import org.firstinspires.ftc.teamcode.util.set

@TeleOp
@Config
@Disabled
class LiftTest : RobotOpMode() {
    companion object {
        @JvmField var liftPower = 0.0
    }

    override suspend fun runSuspendOpMode() {
        suspendUntilStart()

        G.ehub.outtakeLift.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        yieldLooper()
        G.ehub.outtakeLift.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER

        mainLoop {
            telemetry["current"] = G.ehub.outtakeLift.getCurrent(CurrentUnit.AMPS)

            G.ehub.outtakeLift.power = liftPower
            G.ehub.readCurrents()
        }
    }
}