package org.firstinspires.ftc.teamcode.opmodes

import arrow.core.merge
import arrow.fx.coroutines.raceN
import com.outoftheboxrobotics.suspendftc.loopYieldWhile
import com.outoftheboxrobotics.suspendftc.suspendFor
import com.outoftheboxrobotics.suspendftc.suspendUntil
import com.outoftheboxrobotics.suspendftc.yieldLooper
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.util.ElapsedTime
import kotlinx.coroutines.coroutineScope
import kotlinx.coroutines.launch
import kotlinx.coroutines.sync.Mutex
import kotlinx.coroutines.sync.withLock
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import org.firstinspires.ftc.teamcode.RobotState
import org.firstinspires.ftc.teamcode.StateMachine
import org.firstinspires.ftc.teamcode.actions.hardware.ArmPosition
import org.firstinspires.ftc.teamcode.actions.hardware.ClawPosition
import org.firstinspires.ftc.teamcode.actions.hardware.IntakeTiltPosition
import org.firstinspires.ftc.teamcode.actions.hardware.LiftConfig
import org.firstinspires.ftc.teamcode.actions.hardware.TwistPosition
import org.firstinspires.ftc.teamcode.actions.hardware.currentImuAngle
import org.firstinspires.ftc.teamcode.actions.hardware.extensionLength
import org.firstinspires.ftc.teamcode.actions.hardware.intakeTransfer
import org.firstinspires.ftc.teamcode.actions.hardware.liftDown
import org.firstinspires.ftc.teamcode.actions.hardware.liftUpTo
import org.firstinspires.ftc.teamcode.actions.hardware.openClaws
import org.firstinspires.ftc.teamcode.actions.hardware.profileArm
import org.firstinspires.ftc.teamcode.actions.hardware.resetExtensionLength
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
import org.firstinspires.ftc.teamcode.util.set
import org.firstinspires.ftc.teamcode.util.suspendUntilRisingEdge
import org.firstinspires.ftc.teamcode.util.use
import kotlin.math.cos
import kotlin.math.sin

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
                    else {
                        resetExtensionLength()
                        -0.15
                    }
            }

            // Extended
            loopYieldWhile({ extensionLength() > 100 || !C.retractExtension }) {
                G.ehub.extension.power = when {
                    C.extendExtension -> 1.0
                    // Failsafe if encoder loses track
                    G.chub.extensionLimitSwitch -> {
                        resetExtensionLength()
                        -0.15
                    }
                    C.retractExtension -> -1.0
                    extensionLength() < 100 -> 1.0
                    else -> 0.0
                }
            }
        }
    }

    private suspend fun mainRollerJob(): Nothing {
        object : StateMachine {
            // Stopped
            override val defaultState: FS = FS {
                G.ehub.intakeRoller.power = 0.0

                raceN(
                    coroutineContext,
                    {
                        suspendUntil { C.expelRoller && !C.intakeRoller }
                        outtakeState
                    },
                    {
                        suspendUntil { !C.expelRoller && C.intakeRoller }
                        intakeState
                    }
                ).merge()
            }

            private val intakeState: FS = FS {
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
                            suspendUntil { !C.expelRoller && !C.intakeRoller }
                            defaultState
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
                        suspendUntil { !C.expelRoller && !C.intakeRoller }
                        defaultState
                    },
                    {
                        suspendUntil { timer.milliseconds() >= 300 && !C.expelRoller }
                        defaultState
                    }
                ).merge()
            }
        }.run()

        error("Return from mainRollerJob()")
    }

    private val mainState: FS = FS {

        // Prevents state from exiting while held
        val stateExitLock = Mutex()

        launch {
            launch {
                mainLoop {
                    telemetry["Current State"] = "INTAKE STATE"
                }
            }

            while (true) {
                launch {
                    launch { mainExtensionControl() }
                    launch { mainRollerJob() }

                    // TODO Tilt controls
                    setTiltPosition(IntakeTiltPosition.LOW)
                }.use { suspendUntilRisingEdge { C.runTransfer } }

                stateExitLock.withLock {
                    intakeTransfer(finalArmPos = ArmPosition.NEUTRAL.pos, liftEnd = false)
                }
            }
        }.use {
            val targetLiftPos = raceN(
                coroutineContext,
                {
                    suspendUntilRisingEdge { C.outtakeLow }
                    305
                },
                {
                    suspendUntilRisingEdge { C.outtakeHigh }
                    305
                }
            ).merge()

            stateExitLock.withLock {
                coroutineScope {
                    launch { profileArm(ArmPosition.OUTTAKE) }
                    launch { retractExtension() }

                    liftUpTo(targetLiftPos)
                }

                setTwistPosition(TwistPosition.HORIZONTAL)

                outtakeState
            }
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
        G.ehub.extension.power = 0.0
        G.ehub.intakeRoller.power = 0.0

        launch {
            launch {
                mainLoop {
                    telemetry["Current State"] = "OUTTAKE STATE"
                }
            }

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
                        C.liftUp -> LiftConfig.liftUp
                        C.liftDown -> LiftConfig.liftDown
                        else -> LiftConfig.liftHold
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

        liftDown()
        suspendFor(900)
        G.ehub.outtakeLift.power = 0.0

        G.ehub.outtakeLift.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        yieldLooper()
        G.ehub.outtakeLift.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER

        suspendUntilStart()

        launch { runFieldCentricDrive() }
        launch {
            mainLoop {
                val heading = currentImuAngle()
                G.chub.intakeWheel.power =
                    C.driveTurn + C.driveStrafeX * sin(-heading) + C.driveStrafeY * cos(-heading)

                G.ehub.hang0.power = C.hang0
                G.ehub.hang1.power = C.hang1
            }
        }

        runStateMachine(mainState)
    }
}