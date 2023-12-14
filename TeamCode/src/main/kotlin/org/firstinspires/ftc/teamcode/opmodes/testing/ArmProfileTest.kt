package org.firstinspires.ftc.teamcode.opmodes.testing

import arrow.core.merge
import arrow.fx.coroutines.raceN
import com.outoftheboxrobotics.suspendftc.suspendUntil
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import kotlinx.coroutines.coroutineScope
import org.firstinspires.ftc.teamcode.actions.hardware.ArmPosition
import org.firstinspires.ftc.teamcode.actions.hardware.profileArm
import org.firstinspires.ftc.teamcode.opmodes.RobotOpMode
import org.firstinspires.ftc.teamcode.util.merge

@TeleOp
class ArmProfileTest : RobotOpMode() {
    override suspend fun runSuspendOpMode() = coroutineScope {
        suspendUntilStart()

        while (true) {
            val pos = raceN(
                coroutineContext,
                {
                    raceN(
                        coroutineContext,
                        {
                            suspendUntil { gamepad1.x }
                            ArmPosition.TRANSFER
                        },
                        {
                            suspendUntil { gamepad1.y }
                            ArmPosition.NEUTRAL
                        }
                    ).merge()
                },
                {
                    raceN(
                        coroutineContext,
                        {
                            suspendUntil { gamepad1.b }
                            ArmPosition.OUTTAKE
                        },
                        {
                            suspendUntil { gamepad1.a }
                            ArmPosition.AUTON_INIT
                        }
                    ).merge()
                }
            ).merge()

            profileArm(pos)
        }
    }
}