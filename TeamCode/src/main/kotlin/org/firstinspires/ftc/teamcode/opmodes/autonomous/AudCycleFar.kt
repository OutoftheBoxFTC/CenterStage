package org.firstinspires.ftc.teamcode.opmodes.autonomous

import arrow.fx.coroutines.raceN
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.geometry.times
import com.acmerobotics.roadrunner.util.Angle
import com.outoftheboxrobotics.suspendftc.suspendFor
import com.outoftheboxrobotics.suspendftc.suspendUntil
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.util.RobotLog
import kotlinx.coroutines.Job
import kotlinx.coroutines.coroutineScope
import kotlinx.coroutines.flow.drop
import kotlinx.coroutines.flow.filterNotNull
import kotlinx.coroutines.flow.first
import kotlinx.coroutines.launch
import org.firstinspires.ftc.teamcode.RobotState
import org.firstinspires.ftc.teamcode.actions.hardware.ArmPosition
import org.firstinspires.ftc.teamcode.actions.hardware.IntakeTiltPosition
import org.firstinspires.ftc.teamcode.actions.hardware.TwistPosition
import org.firstinspires.ftc.teamcode.actions.hardware.currentDrivePose
import org.firstinspires.ftc.teamcode.actions.hardware.currentImuAngle
import org.firstinspires.ftc.teamcode.actions.hardware.dualFixpoint
import org.firstinspires.ftc.teamcode.actions.hardware.extendoPose
import org.firstinspires.ftc.teamcode.actions.hardware.extensionLength
import org.firstinspires.ftc.teamcode.actions.hardware.ezStopExtension
import org.firstinspires.ftc.teamcode.actions.hardware.followTrajectory
import org.firstinspires.ftc.teamcode.actions.hardware.intakeTransfer
import org.firstinspires.ftc.teamcode.actions.hardware.launchExtensionPid
import org.firstinspires.ftc.teamcode.actions.hardware.launchFixpoint
import org.firstinspires.ftc.teamcode.actions.hardware.launchSmoothStop
import org.firstinspires.ftc.teamcode.actions.hardware.lineTo
import org.firstinspires.ftc.teamcode.actions.hardware.nextImuAngle
import org.firstinspires.ftc.teamcode.actions.hardware.openClaws
import org.firstinspires.ftc.teamcode.actions.hardware.profileArm
import org.firstinspires.ftc.teamcode.actions.hardware.resetDrivePose
import org.firstinspires.ftc.teamcode.actions.hardware.resetImuAngle
import org.firstinspires.ftc.teamcode.actions.hardware.retractExtension
import org.firstinspires.ftc.teamcode.actions.hardware.retractLift
import org.firstinspires.ftc.teamcode.actions.hardware.runExtensionTo
import org.firstinspires.ftc.teamcode.actions.hardware.runImuRecorrection
import org.firstinspires.ftc.teamcode.actions.hardware.scoreOnBackstage
import org.firstinspires.ftc.teamcode.actions.hardware.setDrivePowers
import org.firstinspires.ftc.teamcode.actions.hardware.setExtensionPower
import org.firstinspires.ftc.teamcode.actions.hardware.setTiltPosition
import org.firstinspires.ftc.teamcode.actions.hardware.setTwistPosition
import org.firstinspires.ftc.teamcode.driveState
import org.firstinspires.ftc.teamcode.util.G
import org.firstinspires.ftc.teamcode.util.buildTrajectory
import org.firstinspires.ftc.teamcode.util.deg
import org.firstinspires.ftc.teamcode.util.mainLoop
import org.firstinspires.ftc.teamcode.util.mapState
import org.firstinspires.ftc.teamcode.util.suspendUntilRisingEdge
import org.firstinspires.ftc.teamcode.util.use
import org.firstinspires.ftc.teamcode.vision.PreloadDetectionPipeline.RandomizationPosition
import org.firstinspires.ftc.teamcode.vision.preloadPosition
import org.firstinspires.ftc.teamcode.visionState
import kotlin.math.PI
import kotlin.math.abs

abstract class AudCycleFarAuto(isBlue: Boolean) : AutonOpMode(isBlue, true) {
    abstract val rightPreloadFloor: Pose2d
    abstract val frontPreloadFloor: Pose2d
    abstract val leftPreloadFloor: Pose2d

    abstract val rightPreloadExtension: Vector2d
    abstract val centerPreloadExtension: Vector2d
    abstract val leftPreloadExtension: Vector2d

    abstract val preloadIntakeRobotPos: Pose2d
    abstract val midIntakeExtensionPos: Pose2d

    abstract val preSwoop: Pose2d

    abstract val robotIntakePos: Pose2d
    abstract val robotPreIntakeExtendoPos: Pose2d

    abstract val preOuttake: Pose2d
    abstract val cyclingPreOuttake: Pose2d

    abstract val outtakeRightEstimate: Pose2d
    abstract val outtakeCenterEstimate: Pose2d
    abstract val outtakeLeftEstimate: Pose2d

    private suspend fun intakeFromStack(
        topPos: IntakeTiltPosition,
        nextPos: IntakeTiltPosition?,
        isPreload: Boolean,
        useVision: Boolean,
        cameraScanDelay: Long = 2000
    ) = coroutineScope {
        val intakePos = launch {
            dualFixpoint(
                robotTarget = (if (isPreload) preloadIntakeRobotPos else robotIntakePos).vec(),
                intakeTarget = midIntakeExtensionPos.vec(),
                intakeHeadingConstraintMultiplier = 18.0
            )
        }.use {
            if (useVision) awaitCameraOpen()

            val visionActive = raceN(
                coroutineContext,
                {
                    suspendFor(2000)
                },
                {
                    suspendUntil {
                        if (useVision) tapeDetectionPipeline.inFrame
                        else (G[RobotState.driveState.extendoPose] - midIntakeExtensionPos).vec().norm() < 5.0
                    }
                }
            ).fold(
                ifRight = {
                    true
                },
                ifLeft = {
                    RobotLog.w("Timeout while scanning for tape, not using camera")
                    false
                }
            )



            suspendFor(cameraScanDelay)

            if (useVision && visionActive) {
                val tapePose =
                    G.robotState.mapState { it.visionState.stackTapePose }.filterNotNull().drop(1)
                        .first()

                if ((tapePose - robotPreIntakeExtendoPos).vec().norm() >= 12.0) {
                    RobotLog.i("Tape detection dun goofed: $tapePose")
                    return@use robotPreIntakeExtendoPos
                }

                RobotLog.i("Tape position: $tapePose")

                tapeDetectionPipeline.saveFrameState = true

                tapePose + Pose2d(
                    (tapePose - currentDrivePose()).vec().rotated(PI / 2).let { it / it.norm() },
                    0.0
                )
            } else robotPreIntakeExtendoPos
        }

        startApriltagDetection()

        setTiltPosition(topPos)
        G.ehub.intakeRoller.power = -0.7

        launch {
            dualFixpoint(
                robotTarget = (if (isPreload) preloadIntakeRobotPos else robotIntakePos).vec(),
                intakeTarget = intakePos.vec(),
                intakeHeadingConstraintMultiplier = 10.0
            )
        }.use {
            raceN(
                coroutineContext,
                {
                    suspendFor(2000)
                    RobotLog.w("Timeout while extending to tape")
                },
                {
                    suspendUntil { (G[RobotState.driveState.extendoPose] - intakePos).vec().norm() < 1.0 }
                }
            )
        }

        setDrivePowers(Pose2d())

        G.ehub.intakeWheel.power = 0.0

        setExtensionPower(1.0)
        suspendFor(500)
        ezStopExtension()

        if (nextPos != null) {
            setTiltPosition(nextPos)
            suspendFor(80)
        }
    }

    override suspend fun runSuspendOpMode() = coroutineScope {
        var backstagePos: Pose2d? = null

        fun preloadFloorTrajectory(target: Pose2d) = buildTrajectory(Pose2d()) {
            setReversed(true)
            splineTo(target.vec(), target.heading + PI)
//            lineToLinearHeading(target)
        }

        val preloadRightTrajectory = preloadFloorTrajectory(rightPreloadFloor)
        val preloadCenterTrajectory = preloadFloorTrajectory(frontPreloadFloor)
        val preloadLeftTrajectory = preloadFloorTrajectory(leftPreloadFloor)

        tapeDetectionPipeline.estimate = robotPreIntakeExtendoPos.vec()

        var doCycle = true

        launch {
            launch {
                while (true) {
                    suspendUntilRisingEdge { gamepad1.dpad_down }
                    doCycle = !doCycle
                }
            }
            launch {
                mainLoop {
                    telemetry.addLine(if (doCycle) "RUNNING 2+3" else "RUNNING 2+1")
                }
            }
            runAutonInit()
        }.use {
            retractExtension()
            suspendUntilStart()
        }

        val randomizationPos = G[RobotState.visionState.preloadPosition]

        startTapeDetection()

        resetImuAngle()
        resetDrivePose(Pose2d())
        openClaws()
        setTiltPosition(IntakeTiltPosition.PRELOAD_HOLD)

        // Drive to preload position
        when (randomizationPos) {
            RandomizationPosition.RIGHT -> preloadRightTrajectory
            RandomizationPosition.LEFT -> preloadLeftTrajectory
            RandomizationPosition.CENTER -> preloadCenterTrajectory
        }.let {
            launch { followTrajectory(it) }.use {
                suspendFor((0.6 * it.duration() * 1000).toLong())
            }
        }

        val extendoTarget = when (randomizationPos) {
            RandomizationPosition.LEFT -> leftPreloadExtension
            RandomizationPosition.CENTER -> centerPreloadExtension
            RandomizationPosition.RIGHT -> rightPreloadExtension
        }

        val robotTarget = when (randomizationPos) {
            RandomizationPosition.LEFT -> leftPreloadFloor
            RandomizationPosition.CENTER -> frontPreloadFloor
            RandomizationPosition.RIGHT -> rightPreloadFloor
        }

        val isPastTarget: (Pose2d) -> Boolean = when (randomizationPos) {
            RandomizationPosition.LEFT -> {
                { it.y < extendoTarget.y - 1.0 }
            }
            RandomizationPosition.CENTER -> {
                { it.x > extendoTarget.x + 1.0 }
            }
            RandomizationPosition.RIGHT -> {
                { it.y > extendoTarget.y + 1.0 }
            }
        }

        G.ehub.intakeRoller.power = -0.7

        // Place purple
        launch {
            dualFixpoint(
                intakeTarget = extendoTarget + (extendoTarget - robotTarget.vec()).let { 3.0 * it / it.norm()},
                robotTarget = robotTarget.vec(),
                robotMultiplier = 0.5,
                intakeHeadingConstraintMultiplier = 15.0
            )
        }.use {
            breakpoint()

            suspendUntil {
                currentDrivePose().vec().distTo(robotTarget.vec()) <= 1.0 &&
                isPastTarget(G[RobotState.driveState.extendoPose])
            }

            suspendFor(50)
        }


        launchSmoothStop(robotTarget.copy(heading = (extendoTarget - robotTarget.vec()).angle()))

        setExtensionPower(-0.5)

        suspendUntil { !isPastTarget(G[RobotState.driveState.extendoPose]) }

        setExtensionPower(0.0)
        suspendFor(100)
        setTiltPosition(IntakeTiltPosition.HIGH)
        suspendFor(150)

        retractExtension()

        setTiltPosition(IntakeTiltPosition.POS_1)

        // Drive to pre intake
        if (randomizationPos == RandomizationPosition.LEFT && isBlue) {
            val targetHeading = preloadIntakeRobotPos.heading - PI / 4
            launchSmoothStop(robotTarget.copy(heading = targetHeading))
            suspendUntil { abs(Angle.normDelta(currentDrivePose().heading - targetHeading)) < 5.0.deg }
        }


        launchSmoothStop(preloadIntakeRobotPos)

        runImuRecorrection()

        suspendUntil {
            (currentDrivePose() - preloadIntakeRobotPos).vec().norm() <= 2.0
        }

        intakeFromStack(
            topPos = IntakeTiltPosition.POS_1,
            nextPos = null,
            isPreload = true,
            useVision = true,
            cameraScanDelay = 3000
        )

        if (!doCycle) suspendFor(10_000)

        // Transfer and drive to preswoop :TODO

        runExtensionTo(0, timeout = 1000L, maxAccel = 2000.0)
        retractExtension()

        lineTo(preSwoop, stopDist = 12.0)

        retractExtension()

        coroutineScope {
            launch {
                intakeTransfer(finalLiftPos = 375)

                setTwistPosition(
                    if (randomizationPos == RandomizationPosition.LEFT) TwistPosition.POS_4
                    else TwistPosition.POS_3
                )
            }

            lineTo(preOuttake, stopDist = 0.0)
        }

        breakpoint()

        awaitCameraOpen()

        scoreOnBackstage(
            estimate = when (randomizationPos) {
                RandomizationPosition.LEFT -> outtakeLeftEstimate
                RandomizationPosition.CENTER -> outtakeCenterEstimate
                RandomizationPosition.RIGHT -> outtakeRightEstimate
            },
            target = randomizationPos,
            preOuttake = preOuttake,
            expectedBackstagePos = backstagePos,
            updateBackstagePos = { backstagePos = it },
            precisePlace = true
        )

        if (!doCycle) {
            launchSmoothStop(cyclingPreOuttake)

            mainLoop {
                G.imuStartingHeading = currentImuAngle() + PI
            }
        }

        startTapeDetection()

        // Prepare for cycling
        coroutineScope {
            launch {
                setTwistPosition(TwistPosition.STRAIGHT)
                profileArm(ArmPosition.NEUTRAL)
                retractLift()
            }

            lineTo(preSwoop, stopDist = if (isBlue) 0.0 else 12.0, maxPower = { 0.9 })
            launchSmoothStop(preSwoop)
        }

        breakpoint()

        intakeFromStack(
            topPos = IntakeTiltPosition.POS_2,
            nextPos = IntakeTiltPosition.POS_3,
            isPreload = false,
            useVision = true,
            cameraScanDelay = 1000
        )

        setTwistPosition(TwistPosition.STRAIGHT)

        runExtensionTo(0, timeout = 1000L, maxAccel = 2000.0)
        retractExtension()

        lineTo(preSwoop, stopDist = 12.0)


        coroutineScope {
            launch {
                intakeTransfer(finalLiftPos = 550)
            }

            lineTo(cyclingPreOuttake, stopDist = 0.0)
        }


        breakpoint()

        awaitCameraOpen()

        scoreOnBackstage(
            estimate = if (isBlue) outtakeRightEstimate else outtakeLeftEstimate,
            target = null,
            preOuttake = cyclingPreOuttake,
            expectedBackstagePos = backstagePos,
            updateBackstagePos = { backstagePos = it }
        )

        /* 2+5
        startTapeDetection()

        // Prepare for cycling
        coroutineScope {
            launch {
                setTwistPosition(TwistPosition.STRAIGHT)
                profileArm(ArmPosition.NEUTRAL)
                retractLift()
            }

            lineTo(preSwoop, stopDist = 12.0, maxPower = { 0.9 })
            launchSmoothStop(preSwoop)
        }

        intakeFromStack(
            topPos = IntakeTiltPosition.POS_4,
            nextPos = IntakeTiltPosition.LOW,
            isPreload = false,
            useVision = true
        )

        coroutineScope {
            launch {
                intakeTransfer(finalLiftPos = 550)
                setTwistPosition(TwistPosition.STRAIGHT)
            }

            lineTo(preSwoop, stopDist = 12.0)

            suspendUntil { G.chub.extensionLimitSwitch }

            lineTo(cyclingPreOuttake, stopDist = 0.0)
        }

        breakpoint()

        awaitCameraOpen()

        scoreOnBackstage(
            estimate = if (isBlue) outtakeRightEstimate else outtakeLeftEstimate,
            target = null,
            preOuttake = preOuttake,
            expectedBackstagePos = backstagePos,
            updateBackstagePos = { backstagePos = it }
        )
        */

        launchSmoothStop(cyclingPreOuttake)

        mainLoop {
            G.imuStartingHeading = currentImuAngle() + PI
        }
    }
}

@Autonomous
class RedAudCycleFar : AudCycleFarAuto(false) {
    override val rightPreloadFloor = Pose2d(-49.106, -5.500, 0.622)
    override val rightPreloadExtension = Vector2d(-22.730, 17.275)

    override val frontPreloadFloor = Pose2d(-50.840, 1.416, 5.879)
    override val centerPreloadExtension = Vector2d(-32.0, 0.0)

    override val leftPreloadFloor = Pose2d(-50.848, 2.019, 5.668)
    override val leftPreloadExtension = Vector2d(-28.906, -10.473)

    override val preloadIntakeRobotPos = Pose2d(-52.641, 10.007, 270.0.deg)
    override val midIntakeExtensionPos = Pose2d(-50.023, -6.523, 270.0.deg)

    override val robotIntakePos = Pose2d(-52.640, 30.492, 270.0.deg)
    override val robotPreIntakeExtendoPos = Pose2d(-53.246, -23.214, 270.0.deg)

    override val preSwoop = Pose2d(-53.600, 77.859, 270.0.deg)
    override val preOuttake = Pose2d(-32.2, 73.354, 270.0.deg)

    override val cyclingPreOuttake = Pose2d(-33.0, 76.0, 270.0.deg)

    override val outtakeRightEstimate = Pose2d(-26.978, 81.811,  270.0.deg)
    override val outtakeCenterEstimate = Pose2d(-31.998, 81.593, 270.0.deg)
    override val outtakeLeftEstimate = Pose2d(-38.347, 81.444, 270.0.deg)
}

@Autonomous
class BlueAudCycleFar : AudCycleFarAuto(true) {
    override val rightPreloadFloor = Pose2d(-49.160, -2.459, 0.747)
    override val rightPreloadExtension = Vector2d(-33.817, 12.258)

    override val frontPreloadFloor = Pose2d(-51.113, -2.907, 0.292)
    override val centerPreloadExtension = Vector2d(-32.0, 2.381)

    override val leftPreloadFloor = Pose2d(-42.762, 12.030, 5.430)
    override val leftPreloadExtension = Vector2d(-23.789, -18.093)

    override val preloadIntakeRobotPos = Pose2d(-51.276, -6.986, 90.0.deg)
    override val midIntakeExtensionPos = Pose2d(-50.489, 8.419, 90.0.deg)

    override val robotIntakePos = Pose2d(-52.634, -32.565, 90.0.deg)
    override val robotPreIntakeExtendoPos = Pose2d(-49.583, 27.337, 90.0.deg)

    override val preSwoop =  Pose2d(-52.0, -73.021, 90.deg)
    override val preOuttake = Pose2d(-25.364, -75.677, 90.0.deg)

    override val cyclingPreOuttake = Pose2d(-29.195, -76.544, 90.0.deg)

    override val outtakeRightEstimate = Pose2d(-35.913, -80.464, 90.0.deg)
    override val outtakeCenterEstimate = Pose2d(-30.206, -80.433, 90.0.deg)
    override val outtakeLeftEstimate = Pose2d(-24.019, -80.539, 90.0.deg)
}