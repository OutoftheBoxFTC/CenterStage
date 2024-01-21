package org.firstinspires.ftc.teamcode.opmodes

import arrow.core.merge
import arrow.fx.coroutines.raceN
import com.outoftheboxrobotics.suspendftc.loopYieldWhile
import com.outoftheboxrobotics.suspendftc.suspendFor
import com.outoftheboxrobotics.suspendftc.suspendUntil
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.util.ElapsedTime
import kotlinx.coroutines.coroutineScope
import kotlinx.coroutines.launch
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import org.firstinspires.ftc.teamcode.RobotState
import org.firstinspires.ftc.teamcode.StateMachine
import org.firstinspires.ftc.teamcode.actions.hardware.ArmPosition
import org.firstinspires.ftc.teamcode.actions.hardware.ClawPosition
import org.firstinspires.ftc.teamcode.actions.hardware.IntakeTiltPosition
import org.firstinspires.ftc.teamcode.actions.hardware.TwistPosition
import org.firstinspires.ftc.teamcode.actions.hardware.intakeTransfer
import org.firstinspires.ftc.teamcode.actions.hardware.openClaws
import org.firstinspires.ftc.teamcode.actions.hardware.profileArm
import org.firstinspires.ftc.teamcode.actions.hardware.retractExtension
import org.firstinspires.ftc.teamcode.actions.hardware.runFieldCentricDrive
import org.firstinspires.ftc.teamcode.actions.hardware.setArmPosition
import org.firstinspires.ftc.teamcode.actions.hardware.setClawPos
import org.firstinspires.ftc.teamcode.actions.hardware.setTiltPosition
import org.firstinspires.ftc.teamcode.actions.hardware.setTwistPosition
import org.firstinspires.ftc.teamcode.actions.hardware.twistPos
import org.firstinspires.ftc.teamcode.opmodes.testing.IntakeTest
import org.firstinspires.ftc.teamcode.outtakeState
import org.firstinspires.ftc.teamcode.runStateMachine
import org.firstinspires.ftc.teamcode.util.C
import org.firstinspires.ftc.teamcode.util.FS
import org.firstinspires.ftc.teamcode.util.G
import org.firstinspires.ftc.teamcode.util.mainLoop
import org.firstinspires.ftc.teamcode.util.suspendUntilRisingEdge
import org.firstinspires.ftc.teamcode.util.use

/**
 * Main Teleop program.
 */
@TeleOp
class MainTeleop : RobotOpMode() {
    private suspend fun mainExtensionControl(): Nothing {
        while (true) {
            // Retracted
            loopYieldWhile({ !C.extendExtension }) {
                G.ehub.extension.power =
                    if (!G.chub.extensionLimitSwitch) -1.0
                    else -0.15
            }

            // Extended
            loopYieldWhile({ G.ehub.extension.currentPosition > 100 || !C.retractExtension }) {
                G.ehub.extension.power = when {
                    C.retractExtension -> -1.0
                    C.extendExtension -> 1.0
                    G.ehub.extension.currentPosition < 100 -> 1.0
                    else -> 0.0
                }
            }
        }
    }

    private suspend fun mainRollerJob(): Nothing {
        object : StateMachine {
            // Intaking
            override val defaultState: FS = FS {
                launch {
                    while (true) {
                        G.ehub.readCurrents()

                        if (G.ehub.intakeRoller.getCurrent(CurrentUnit.AMPS) > IntakeTest.maxRollerCurrent) {
                            G.ehub.intakeRoller.power = 1.0
                            suspendFor(20)
                            G.ehub.intakeRoller.power = -1.0
                        } else G.ehub.intakeRoller.power = -1.0

                        suspendFor(200)
                    }
                }.use {
                    raceN(
                        coroutineContext,
                        {
                            suspendUntil { C.expelRoller }
                            outtakeState
                        },
                        {
                            suspendUntil { C.stopRoller }
                            stopState
                        }
                    ).merge()
                }
            }

            private val outtakeState: FS = FS {
                G.ehub.intakeRoller.power = 0.7
                val timer = ElapsedTime()
                raceN(
                    coroutineContext,
                    {
                        suspendUntil { C.stopRoller }
                        stopState
                    },
                    {
                        suspendUntil { timer.milliseconds() >= 300 && !C.expelRoller }
                        defaultState
                    }
                ).merge()
            }

            private val stopState: FS = FS {
                G.ehub.intakeRoller.power = 0.0

                raceN(
                    coroutineContext,
                    {
                        suspendUntil { C.expelRoller && !C.stopRoller }
                        outtakeState
                    },
                    {
                        suspendUntil { !C.expelRoller && !C.stopRoller }
                        defaultState
                    }
                ).merge()
            }
        }.run()

        error("Return from mainRollerJob()")
    }

    private val mainState: FS = FS {
        launch {
            while (true) {
                launch {
                    launch { mainExtensionControl() }
                    launch { mainRollerJob() }

                    // TODO Tilt controls
                    setTiltPosition(IntakeTiltPosition.LOW)
                }.use { suspendUntilRisingEdge { C.runTransfer } }

                intakeTransfer()
            }
        }.use {
            suspendUntilRisingEdge { C.outtakeLow || C.outtakeHigh }
            coroutineScope {
                launch { profileArm(ArmPosition.OUTTAKE) }
                launch { retractExtension() }
                // TODO Move lift to target
            }
            outtakeState
        }
    }

    // Twist position toggling
    private fun nextTwistPos() {
        val positions = TwistPosition.entries
        G[RobotState.outtakeState.twistPos].ordinal.let {
            if (it+1 >= positions.size) 0
            else it+1
        }.let {
            setTwistPosition(positions[it])
        }
    }

    private fun prevTwistPos() {
        val positions = TwistPosition.entries
        G[RobotState.outtakeState.twistPos].ordinal.let {
            if (it-1 < 0) positions.size - 1
            else it-1
        }.let {
            setTwistPosition(positions[it])
        }
    }

    // Claw release
    private fun releaseLeft() {
        val currentPos = G[RobotState.outtakeState.twistPos].ordinal

        val targetPos = if (currentPos < TwistPosition.STRAIGHT.ordinal) ClawPosition.RED_OPEN
        else ClawPosition.BLACK_OPEN

        setClawPos(targetPos)
    }

    private fun releaseRight() {
        val currentPos = G[RobotState.outtakeState.twistPos].ordinal

        val targetPos = if (currentPos < TwistPosition.STRAIGHT.ordinal) ClawPosition.BLACK_OPEN
        else ClawPosition.RED_OPEN

        setClawPos(targetPos)
    }

    private val outtakeState: FS = FS {
        launch {
            // Claw twisting
            launch {
                while (true) {
                    raceN(
                        coroutineContext,
                        {
                            suspendUntilRisingEdge { C.twistClawRight }
                            nextTwistPos()
                        },
                        {
                            suspendUntilRisingEdge { C.twistClawLeft }
                            prevTwistPos()
                        }
                    )
                }
            }

            launch {
                mainLoop {
                    if (C.releaseLeftClaw) releaseLeft()
                    if (C.releaseRightClaw) releaseRight()

                    G.ehub.outtakeLift.power = when {
                        C.liftUp -> 1.0
                        C.liftDown -> -0.5
                        else -> 0.5
                    }
                }
            }
        }.use {
            suspendUntilRisingEdge { C.exitOuttake }
        }

        G.ehub.outtakeLift.power = -1.0
        setTwistPosition(TwistPosition.STRAIGHT)
        profileArm(ArmPosition.NEUTRAL)
        G.ehub.outtakeLift.power = 0.0

        mainState
    }

    override suspend fun runSuspendOpMode() = coroutineScope {
        setArmPosition(ArmPosition.NEUTRAL)
        setTwistPosition(TwistPosition.STRAIGHT)
        setTiltPosition(IntakeTiltPosition.HIGH)
        openClaws()

        retractExtension()

        suspendUntilStart()

        launch { runFieldCentricDrive() }
        launch {
            mainLoop {
                G.chub.intakeWheel.power = C.driveTurn
                G.ehub.hang0.power = C.hang0
                G.ehub.hang1.power = C.hang1
            }
        }

        runStateMachine(mainState)
    }
}