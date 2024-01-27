package org.firstinspires.ftc.teamcode.opmodes

import arrow.core.merge
import arrow.core.nel
import arrow.fx.coroutines.raceN
import com.outoftheboxrobotics.suspendftc.loopYieldWhile
import com.outoftheboxrobotics.suspendftc.suspendFor
import com.outoftheboxrobotics.suspendftc.suspendUntil
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import kotlinx.coroutines.Job
import kotlinx.coroutines.async
import kotlinx.coroutines.cancelAndJoin
import kotlinx.coroutines.launch
import org.firstinspires.ftc.teamcode.Command
import org.firstinspires.ftc.teamcode.RobotState
import org.firstinspires.ftc.teamcode.Subsystem
import org.firstinspires.ftc.teamcode.actions.hardware.ArmPosition
import org.firstinspires.ftc.teamcode.actions.hardware.ClawPosition
import org.firstinspires.ftc.teamcode.actions.hardware.IntakeTiltPosition
import org.firstinspires.ftc.teamcode.actions.hardware.TwistPosition
import org.firstinspires.ftc.teamcode.actions.hardware.closeClaws
import org.firstinspires.ftc.teamcode.actions.hardware.currentImuAngle
import org.firstinspires.ftc.teamcode.actions.hardware.extensionLength
import org.firstinspires.ftc.teamcode.actions.hardware.openClaws
import org.firstinspires.ftc.teamcode.actions.hardware.profileArm
import org.firstinspires.ftc.teamcode.actions.hardware.retractExtension
import org.firstinspires.ftc.teamcode.actions.hardware.runFieldCentricDrive
import org.firstinspires.ftc.teamcode.actions.hardware.setArmPosition
import org.firstinspires.ftc.teamcode.actions.hardware.setClawPos
import org.firstinspires.ftc.teamcode.actions.hardware.setDrivePowers
import org.firstinspires.ftc.teamcode.actions.hardware.setExtensionPower
import org.firstinspires.ftc.teamcode.actions.hardware.setTiltPosition
import org.firstinspires.ftc.teamcode.actions.hardware.setTwistPosition
import org.firstinspires.ftc.teamcode.actions.hardware.twistPos
import org.firstinspires.ftc.teamcode.outtakeState
import org.firstinspires.ftc.teamcode.runStateMachine
import org.firstinspires.ftc.teamcode.util.C
import org.firstinspires.ftc.teamcode.util.FS
import org.firstinspires.ftc.teamcode.util.G
import org.firstinspires.ftc.teamcode.util.mainLoop
import org.firstinspires.ftc.teamcode.util.suspendUntilRisingEdge
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.sin

/**
 * Main Teleop program.
 */
@TeleOp
class MainTeleop : RobotOpMode() {
    private val fieldCentricCommand = Command(Subsystem.DRIVETRAIN.nel()) {
        runFieldCentricDrive()
    }

    /**
     * Main state while driving to intake or outtake.
     *
     * @param transitionJob The job to wait for before transitioning to the next state.
     */
    private fun mainState(transitionJob: Job? = null): FS = FS {
        launch { G.cmd.runCommand(fieldCentricCommand) }

        transitionJob?.join()

        // Next state is determined by which button is pressed first.
        val nextState = async {
            raceN(
                coroutineContext,
                {
                    suspendUntil { C.enterIntakeState }
                    intakeState
                },
                {
                    suspendUntil { C.enterOuttakeState }
                    outtakeState
                }
            ).merge()
        }

        // Operate hang servos
        loopYieldWhile({ !nextState.isCompleted }) {
            G.ehub.hang0.power = C.hang0
            G.ehub.hang1.power = C.hang1
        }

        nextState.await()
    }

    /**
     * State for intaking pixels.
     */
    private val intakeState: FS = FS {
        openClaws()

        launch {
            // Modified field centric drive that gives operator control over turning.
            G.cmd.runNewCommand(Subsystem.DRIVETRAIN.nel()) {
                mainLoop {
                    val heading = currentImuAngle()

                    val multiplier = if (C.slowDrive) 0.4 else 1.0

                    setDrivePowers(
                        multiplier * (C.driveStrafeX * cos(-heading) - C.driveStrafeY * sin(-heading)),
                        multiplier * (C.driveStrafeX * sin(-heading) + C.driveStrafeY * cos(-heading)),
                        if (extensionLength() > 100 && C.operatorIntakeExtension <= 0.5)
                            0.7 * C.operatorTurn else multiplier * C.driveTurn
                    )
                }
            }
        }

        // TODO add Tilt Controls
        setTiltPosition(IntakeTiltPosition.LOW)

        // Operator intake control
        loopYieldWhile({!C.enterMainState && !C.exitIntakeNoTransfer}) {
            setExtensionPower(C.operatorIntakeExtension.let {
                if (abs(it) < 0.1 && extensionLength() < 100) -0.15 else it
            })

            G.chub.intakeWheel.power = C.operatorTurn

            when {
                C.rollerIn -> -1.0
                C.rollerOut -> 1.0
                else -> 0.0
            }.let { G.ehub.intakeRoller.power = it }
        }

        val transferJob: Job = if (C.exitIntakeNoTransfer) launch {
            retractExtension()
            setTiltPosition(IntakeTiltPosition.HIGH)
        } else launch {
            // Transfer sequence
            G.ehub.intakeRoller.power = -0.8
            setTiltPosition(IntakeTiltPosition.HIGH)
            retractExtension()
            suspendFor(1000)
            profileArm(ArmPosition.TRANSFER)
            G.ehub.intakeRoller.power = 0.0
            closeClaws()
            suspendFor(100)
            setTiltPosition(IntakeTiltPosition.LOW)
            suspendFor(50)
            setArmPosition(ArmPosition.NEUTRAL)
            suspendFor(200)
            setTiltPosition(IntakeTiltPosition.HIGH)
        }

        mainState(transferJob)
    }

    /**
     * State for outtaking pixels.
     */
    private val outtakeState: FS = FS {
        launch { G.cmd.runCommand(fieldCentricCommand) }

        profileArm(ArmPosition.OUTTAKE)
        setTwistPosition(TwistPosition.HORIZONTAL)

        // Twist position toggling
        fun nextTwistPos() {
            val positions = TwistPosition.entries
            G[RobotState.outtakeState.twistPos].ordinal.let {
                if (it+1 >= positions.size) 0
                else it+1
            }.let {
                setTwistPosition(positions[it])
            }
        }

        fun prevTwistPos() {
            val positions = TwistPosition.entries
            G[RobotState.outtakeState.twistPos].ordinal.let {
                if (it-1 < 0) positions.size - 1
                else it-1
            }.let {
                setTwistPosition(positions[it])
            }
        }

        fun releaseFar() {
            val currentPos = G[RobotState.outtakeState.twistPos].ordinal

            val targetPos = if (currentPos < TwistPosition.STRAIGHT.ordinal) ClawPosition.RED_OPEN
            else ClawPosition.BLACK_OPEN

            setClawPos(targetPos)
        }

        fun releaseClose() {
            val currentPos = G[RobotState.outtakeState.twistPos].ordinal

            val targetPos = if (currentPos < TwistPosition.STRAIGHT.ordinal) ClawPosition.BLACK_OPEN
            else ClawPosition.RED_OPEN

            setClawPos(targetPos)
        }

        // Claw rotation
        val armJob = launch {
            launch {
                mainLoop {
                    suspendUntilRisingEdge { C.clawRotateRight }
                    nextTwistPos()
                }
            }

            launch {
                mainLoop {
                    suspendUntilRisingEdge { C.clawRotateLeft }
                    prevTwistPos()
                }
            }
        }

        loopYieldWhile({ !C.enterMainState }) {
            // Operator claw release and lift control
            if (C.releaseFarPixel) releaseFar()
            if (C.releaseClosePixel) releaseClose()

            G.ehub.outtakeLift.power = when {
                C.liftUp -> 1.0
                C.liftDown -> 0.5
                else -> -0.5
            }
        }

        mainState(launch {
            G.ehub.outtakeLift.power = 1.0
            armJob.cancelAndJoin()
            openClaws()
            setTwistPosition(TwistPosition.STRAIGHT)
            profileArm(ArmPosition.NEUTRAL)
            G.ehub.outtakeLift.power = 0.0
        })
    }

    override suspend fun runSuspendOpMode() {
        setArmPosition(ArmPosition.NEUTRAL)
        setTwistPosition(TwistPosition.STRAIGHT)
        setTiltPosition(IntakeTiltPosition.HIGH)
        openClaws()

        retractExtension()

        suspendUntilStart()

        runStateMachine(mainState())
    }
}