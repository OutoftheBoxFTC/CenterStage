package org.firstinspires.ftc.teamcode.opmodes

import arrow.core.merge
import arrow.fx.coroutines.raceN
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.outoftheboxrobotics.suspendftc.loopYieldWhile
import com.outoftheboxrobotics.suspendftc.suspendFor
import com.outoftheboxrobotics.suspendftc.suspendUntil
import com.outoftheboxrobotics.suspendftc.yieldLooper
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.util.ElapsedTime
import kotlinx.coroutines.Job
import kotlinx.coroutines.coroutineScope
import kotlinx.coroutines.launch
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import org.firstinspires.ftc.teamcode.RobotState
import org.firstinspires.ftc.teamcode.StateMachine
import org.firstinspires.ftc.teamcode.actions.controllers.PidCoefs
import org.firstinspires.ftc.teamcode.actions.controllers.runPosePidController
import org.firstinspires.ftc.teamcode.actions.hardware.ArmPosition
import org.firstinspires.ftc.teamcode.actions.hardware.DriveConfig
import org.firstinspires.ftc.teamcode.actions.hardware.IntakeTiltPosition
import org.firstinspires.ftc.teamcode.actions.hardware.LiftConfig
import org.firstinspires.ftc.teamcode.actions.hardware.TwistPosition
import org.firstinspires.ftc.teamcode.actions.hardware.closeClaws
import org.firstinspires.ftc.teamcode.actions.hardware.closeDrone
import org.firstinspires.ftc.teamcode.actions.hardware.currentDrivePose
import org.firstinspires.ftc.teamcode.actions.hardware.currentImuAngle
import org.firstinspires.ftc.teamcode.actions.hardware.extensionLength
import org.firstinspires.ftc.teamcode.actions.hardware.intakeTransfer
import org.firstinspires.ftc.teamcode.actions.hardware.launchDrone
import org.firstinspires.ftc.teamcode.actions.hardware.liftUpTo
import org.firstinspires.ftc.teamcode.actions.hardware.openClaws
import org.firstinspires.ftc.teamcode.actions.hardware.profileArm
import org.firstinspires.ftc.teamcode.actions.hardware.resetExtensionLength
import org.firstinspires.ftc.teamcode.actions.hardware.resetImuAngle
import org.firstinspires.ftc.teamcode.actions.hardware.retractExtension
import org.firstinspires.ftc.teamcode.actions.hardware.retractLift
import org.firstinspires.ftc.teamcode.actions.hardware.runFieldCentricDrive
import org.firstinspires.ftc.teamcode.actions.hardware.setArmPosition
import org.firstinspires.ftc.teamcode.actions.hardware.setDrivePowers
import org.firstinspires.ftc.teamcode.actions.hardware.setTiltPosition
import org.firstinspires.ftc.teamcode.actions.hardware.setTwistPosition
import org.firstinspires.ftc.teamcode.actions.hardware.twistPos
import org.firstinspires.ftc.teamcode.opmodes.testing.IntakeTest
import org.firstinspires.ftc.teamcode.outtakeState
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.runStateMachine
import org.firstinspires.ftc.teamcode.util.C
import org.firstinspires.ftc.teamcode.util.FS
import org.firstinspires.ftc.teamcode.util.G
import org.firstinspires.ftc.teamcode.util.mainLoop
import org.firstinspires.ftc.teamcode.util.suspendUntilRisingEdge
import org.firstinspires.ftc.teamcode.util.use
import kotlin.coroutines.coroutineContext
import kotlin.math.PI
import kotlin.math.cos
import kotlin.math.sign
import kotlin.math.sin

/**
 * Main Teleop program.
 */
@TeleOp
class MainTeleop : RobotOpMode() {
    private var requireTransferAux = false

    private val mainIntakeState: FS = FS {
        color(0.0, 1.0, 0.0)

        gamepad1.rumble(200)

        setTwistPosition(TwistPosition.STRAIGHT)
        setArmPosition(ArmPosition.NEUTRAL)
        setTiltPosition(IntakeTiltPosition.LOW)

        launch {
            launch { defaultDriveControl() }
            launch { driverExtensionControl() }
            launch { defaultRollerControl() }
        }.mainOperatorControlNextState(operatorControlState)
    }

    private val operatorControlState: FS = FS {
        color(1.0, 0.0, 0.0)

        gamepad2.rumble(Gamepad.RUMBLE_DURATION_CONTINUOUS)

        setTwistPosition(TwistPosition.STRAIGHT)
        setArmPosition(ArmPosition.NEUTRAL)
        setTiltPosition(IntakeTiltPosition.LOW)

        launch {
            launch { runFieldCentricDrive() }
            launch { defaultRollerControl() }
            launch { operatorTiltControl() }
            launch {
                mainLoop {
                    G.ehub.extension.power = C.operatorExtension.toDouble()
                    G.chub.intakeWheel.power = C.operatorPivot.toDouble()
                }
            }
        }.mainOperatorControlNextState(mainIntakeState).also {
            gamepad2.stopRumble()
        }
    }

    private val transferState: FS = FS {
        requireTransferAux = true

        color(1.0, 1.0, 0.0)

        launch {
            launch { defaultDriveControl() }
        }.use {
            raceN(
                coroutineContext,
                {
                    suspendUntilRisingEdge { C.quitTransfer }
                },
                {
                    intakeTransfer(finalArmPos = ArmPosition.NEUTRAL.pos, liftEnd = false)
                }
            ).onRight {
                setArmPosition(ArmPosition.NEUTRAL)
                setTiltPosition(IntakeTiltPosition.LOW)
                retractLift()
            }

            mainIntakeState
        }
    }

    private val outtakeState: FS = FS {
        requireTransferAux = false

        color(0.0, 0.0, 1.0)

        setArmPosition(ArmPosition.OUTTAKE)
        setTiltPosition(IntakeTiltPosition.HIGH)
        G.ehub.intakeRoller.power = 0.0

        var enableLift = true

        launch {
            launch { defaultDriveControl() }
            launch { retractExtension() }
            launch {
                mainLoop {
                    if (enableLift) G.ehub.outtakeLift.power = when {
                        C.operatorLiftUp -> LiftConfig.liftUp
                        C.operatorLiftDown -> LiftConfig.liftDown
                        else -> LiftConfig.liftHold
                    }
                }
            }
            launch {
                suspendUntilRisingEdge { C.releaseClaw }
                openClaws()
            }
        }.use {
            raceN(
                coroutineContext,
                {
                    suspendUntil { C.autoAlign }
                    autoAlignState
                },
                {
                    suspendUntilRisingEdge { C.exitOuttake || C.startRoller || C.reverseRoller }
                    mainIntakeState
                }
            ).onRight {
                setTwistPosition(TwistPosition.STRAIGHT)

                coroutineScope {
                    launch { profileArm(ArmPosition.NEUTRAL) }

                    enableLift = false
                    retractLift()
                }
            }.merge()
        }
    }

    private val autoAlignState: FS = FS {
        color(1.0, 0.0, 1.0)

        gamepad1.rumble(Gamepad.RUMBLE_DURATION_CONTINUOUS)

        setTwistPosition(TwistPosition.HORIZONTAL)
        setArmPosition(ArmPosition.OUTTAKE)
        setTiltPosition(IntakeTiltPosition.HIGH)
        G.ehub.intakeRoller.power = 0.0

        launch {
            launch { driverTwistControl() }
            launch { retractExtension() }

            var targetHeading = PI / 2 * sign(currentImuAngle())
            var headingPower = 0.0

            launch {
                runPosePidController(
                    translationalCoefs = PidCoefs(0.0, 0.0, 0.0),
                    headingCoefs = DriveConfig.headingPid,
                    input = { currentDrivePose() },
                    target = { Pose2d(0.0, 0.0, targetHeading) },
                    output = { headingPower = it.heading }
                )
            }

            launch {
                val timer = ElapsedTime()

                yieldLooper()

                mainLoop {
                    val dt = timer.seconds()
                    timer.reset()

                    targetHeading += 0.1 * C.driveHeadingAdjust * dt
                }
            }

            launch {
                mainLoop {
                    G.ehub.outtakeLift.power =
                        LiftConfig.liftHold + (0.4 * C.driveAutoLift).let {
                            if (it > 0.0) it * 1.8 else it
                        }

                    val heading = currentImuAngle()
                    val driveX = 0.5 * C.driveStrafeX
                    val driveY = 0.3 * C.driveStrafeY

                    setDrivePowers(
                        driveX * cos(-heading) - driveY * sin(-heading),
                        SampleMecanumDrive.LATERAL_MULTIPLIER * (driveX * sin(-heading) + driveY * cos(-heading)),
                        headingPower
                    )
                }
            }

        }.use {
            suspendUntil { !C.autoAlign }
            gamepad1.stopRumble()
            openClaws()
            outtakeState
        }
    }

    private suspend fun defaultDriveControl(): Nothing = coroutineScope {
        launch { runFieldCentricDrive() }

        mainLoop {
            val heading = currentImuAngle()
            G.chub.intakeWheel.power =
                C.driveTurn + C.driveStrafeX * sin(-heading) + C.driveStrafeY * cos(-heading)
        }
    }

    private suspend fun driverExtensionControl(): Nothing {
        var startExtended = extensionLength() > 100

        while (true) {
            // Retracted
            if (!startExtended) {
                startExtended = false

                loopYieldWhile({ !C.driverExtend }) {
                    G.ehub.extension.power =
                        if (!G.chub.extensionLimitSwitch) -1.0
                        else {
                            resetExtensionLength()
                            -0.15
                        }
                }
            }

            // Extended
            loopYieldWhile({ extensionLength() > 100 || !C.driverRetract }) {
                G.ehub.extension.power = when {
                    C.driverExtend -> 1.0
                    // Failsafe if encoder loses track
                    G.chub.extensionLimitSwitch -> {
                        resetExtensionLength()
                        -0.15
                    }
                    C.driverRetract -> -1.0
                    extensionLength() < 100 -> 1.0
                    else -> 0.0
                }
            }
        }
    }

    private suspend fun defaultRollerControl(): Nothing {
        object : StateMachine {
            // Stopped
            override val defaultState: FS = FS {
                G.ehub.intakeRoller.power = 0.0

                raceN(
                    coroutineContext,
                    {
                        suspendUntil { C.reverseRoller && !C.startRoller }
                        outtakeState
                    },
                    {
                        suspendUntil { !C.reverseRoller && C.startRoller }
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
                            suspendUntil { C.reverseRoller }
                            outtakeState
                        },
                        {
                            suspendUntil { !C.reverseRoller && !C.startRoller }
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
                        suspendUntil { !C.reverseRoller && !C.startRoller }
                        defaultState
                    },
                    {
                        suspendUntil { timer.milliseconds() >= 300 && !C.reverseRoller }
                        defaultState
                    }
                ).merge()
            }
        }.run()

        error("Return from defaultRollerControl()")
    }

    private suspend fun operatorTiltControl(): Nothing {
        var tiltPos = IntakeTiltPosition.LOW

        setTiltPosition(tiltPos)

        fun posFromOrd(ord: Int): IntakeTiltPosition {
            val entries = IntakeTiltPosition.entries
            val size = entries.size

            return when {
                ord < 5 -> entries[size - (5 - ord)]
                ord >= size -> entries[5 + ord - size]
                else -> entries[ord]
            }
        }

        while (true) {
            raceN(
                coroutineContext,
                {
                    suspendUntilRisingEdge { C.tiltToggleUp }
                    posFromOrd(tiltPos.ordinal - 1)
                },
                {
                    suspendUntilRisingEdge { C.tiltToggleDown }
                    posFromOrd(tiltPos.ordinal + 1)
                }
            ).merge().let {
                setTiltPosition(it)
                tiltPos = it
            }
        }
    }

    private suspend fun driverTwistControl(): Nothing {
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

        while (true) {
            raceN(
                coroutineContext,
                {
                    suspendUntilRisingEdge { C.driveTwistRight }
                    nextTwistPos()
                },
                {
                    suspendUntilRisingEdge { C.driveTwistLeft }
                    prevTwistPos()
                }
            )
        }
    }

    private suspend fun Job.mainOperatorControlNextState(nextOperatorToggleState: FS): FS {
        return use {
            raceN(
                coroutineContext,
                {
                    suspendUntil { C.enterTransfer && (!requireTransferAux || C.enterTransferAux) }
                    transferState
                },
                {
                    suspendUntilRisingEdge { C.toggleOperatorOverride }
                    nextOperatorToggleState
                },
                {
                    raceN(
                        coroutineContext,
                        {
                            suspendUntilRisingEdge { C.outtakeLow }
                            360
                        },
                        {
                            suspendUntilRisingEdge { C.outtakeHigh }
                            500
                        }
                    ).merge()
                }
            )
        }.fold(
            { it },
            { it },
            { liftPos ->
                coroutineScope {
                    launch {
                        launch { profileArm(ArmPosition.OUTTAKE) }
                        launch { retractExtension() }

                        raceN(
                            coroutineContext,
                            {
                                suspendFor(1500)
                            },
                            {
                                liftUpTo(liftPos)
                            }
                        )
                    }

                    suspendFor(250)

                    setTwistPosition(TwistPosition.HORIZONTAL)
                    closeClaws()

                    outtakeState
                }
            }
        )
    }

    private fun rumble(millis: Int) {
        gamepad1.rumble(millis)
        gamepad2.rumble(millis)
    }

    private fun color(r: Double, g: Double, b: Double, duration: Int = Gamepad.LED_DURATION_CONTINUOUS) {
        gamepad1.setLedColor(r, g, b, duration)
        gamepad2.setLedColor(r, g, b, duration)
    }

    override suspend fun runSuspendOpMode() = coroutineScope {
        G.imuStartingHeading?.let { resetImuAngle(it) }
        G.imuStartingHeading = null

        setArmPosition(ArmPosition.NEUTRAL)
        setTwistPosition(TwistPosition.STRAIGHT)
        setTiltPosition(IntakeTiltPosition.HIGH)
        closeDrone()
        openClaws()

        retractExtension()

        G.ehub.outtakeLift.power = -1.0
        suspendFor(900)
        G.ehub.outtakeLift.power = 0.0

        G.ehub.outtakeLift.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        yieldLooper()
        G.ehub.outtakeLift.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER

        rumble(400)

        suspendUntilStart()

        launch {
            mainLoop {
                G.ehub.hang0.power = C.hang0
                G.ehub.hang1.power = C.hang1

                if (C.launchDrone) launchDrone() else closeDrone()
            }
        }

        runStateMachine(mainIntakeState)
    }
}