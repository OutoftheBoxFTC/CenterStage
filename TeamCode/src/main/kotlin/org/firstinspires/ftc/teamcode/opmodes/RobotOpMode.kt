package org.firstinspires.ftc.teamcode.opmodes

import arrow.fx.coroutines.autoCloseable
import arrow.fx.coroutines.resourceScope
import arrow.optics.copy
import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.outoftheboxrobotics.suspendftc.Looper
import com.outoftheboxrobotics.suspendftc.loopYieldWhile
import com.outoftheboxrobotics.suspendftc.suspendFor
import com.outoftheboxrobotics.suspendftc.suspendUntil
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.IMU
import kotlinx.coroutines.DelicateCoroutinesApi
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.ExperimentalCoroutinesApi
import kotlinx.coroutines.isActive
import kotlinx.coroutines.launch
import kotlinx.coroutines.newSingleThreadContext
import kotlinx.coroutines.runBlocking
import kotlinx.coroutines.withContext
import org.firstinspires.ftc.teamcode.Globals
import org.firstinspires.ftc.teamcode.Globals.defaultRobotState
import org.firstinspires.ftc.teamcode.RobotState
import org.firstinspires.ftc.teamcode.actions.hardware.ArmPosition
import org.firstinspires.ftc.teamcode.actions.hardware.DriveControlState
import org.firstinspires.ftc.teamcode.actions.hardware.IntakeTiltPosition
import org.firstinspires.ftc.teamcode.actions.hardware.TwistPosition
import org.firstinspires.ftc.teamcode.actions.hardware.closeClaws
import org.firstinspires.ftc.teamcode.actions.hardware.openClaws
import org.firstinspires.ftc.teamcode.actions.hardware.profileArm
import org.firstinspires.ftc.teamcode.actions.hardware.resetDrivePose
import org.firstinspires.ftc.teamcode.actions.hardware.runDefaultImuHandler
import org.firstinspires.ftc.teamcode.actions.hardware.runRoadrunnerDrivetrain
import org.firstinspires.ftc.teamcode.actions.hardware.runThreadedImuHandler
import org.firstinspires.ftc.teamcode.actions.hardware.setArmPosition
import org.firstinspires.ftc.teamcode.actions.hardware.setTiltPosition
import org.firstinspires.ftc.teamcode.actions.hardware.setTwistPosition
import org.firstinspires.ftc.teamcode.driveLooper
import org.firstinspires.ftc.teamcode.driveState
import org.firstinspires.ftc.teamcode.hardware.IMU_NAME
import org.firstinspires.ftc.teamcode.logState
import org.firstinspires.ftc.teamcode.mainLooper
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive

abstract class RobotOpMode(
    private val runMultiThreaded: Boolean = true,
    private val monitorOpmodeStop: Boolean = true,
    private val imuRunMode: ImuRunMode = ImuRunMode.THREADED,
    private val initializeServos: Boolean = false,
    private val resetPoseOnStart: Boolean = true
) : LinearOpMode() {
    enum class ImuRunMode {
        NONE,
        DEFAULT,
        THREADED
    }

    abstract suspend fun runSuspendOpMode()

    protected var manualStop = false

    protected suspend fun suspendUntilStart() {
        withContext(Dispatchers.IO) {
            waitForStart()
        }

        if (resetPoseOnStart) resetDrivePose(Pose2d())
    }

    private suspend fun initializeServos() {
        openClaws()
        setTwistPosition(TwistPosition.STRAIGHT)
        setArmPosition(ArmPosition.NEUTRAL)
        setTiltPosition(IntakeTiltPosition.LOW)
        suspendFor(500)
        profileArm(ArmPosition.TRANSFER)
        suspendFor(200)
        setTiltPosition(IntakeTiltPosition.HIGH)
        suspendFor(400)
        closeClaws()
        suspendFor(300)
        setTiltPosition(IntakeTiltPosition.LOW)
    }

    @OptIn(ExperimentalCoroutinesApi::class, DelicateCoroutinesApi::class)
    override fun runOpMode() {
        manualStop = false

        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
        telemetry.msTransmissionInterval = 15

        val mainLooperLens = RobotState.mainLooper
        val driveLooperLens = RobotState.driveLooper

        defaultRobotState(hardwareMap).copy {
            if (runMultiThreaded) { driveLooperLens set Looper() }
        }.let { Globals.initializeRobotState(it, this) }

        val rrDrive = SampleMecanumDrive(hardwareMap)

        Globals[driveLooperLens].scheduleCoroutine {
            launch {
                runRoadrunnerDrivetrain(rrDrive)
            }

            loopYieldWhile({ isActive }) {
                Globals.chub.syncHardware()
            }
        }

        Globals[mainLooperLens].scheduleCoroutine {
            loopYieldWhile({ isActive }) {
                Globals.ehub.syncHardware()

                val driveState = Globals[RobotState.driveState]

                driveState.rrDrive?.let {
                    val packet = it.getDriveTelemetryPacket(
                        (driveState.driveControlState as? DriveControlState.Fixpoint)?.target
                    )

                    FtcDashboard.getInstance().sendTelemetryPacket(packet)
                }

                telemetry.logState(Globals.robotState.value)
                telemetry.update()
            }
        }

        Globals[mainLooperLens].scheduleCoroutine {
            val job = launch {
                if (initializeServos) initializeServos()
                runSuspendOpMode()
            }

            suspendUntil {
                manualStop || job.isCompleted || (monitorOpmodeStop && !opModeIsActive() && !opModeInInit())
            }

            Globals.stop()
        }

        val driveThread = autoCloseable { newSingleThreadContext("Drive thread") }
        val imuThread = autoCloseable { newSingleThreadContext("IMU thread") }

        runBlocking {
            resourceScope {
                if (imuRunMode != ImuRunMode.NONE) {
                    val imu = hardwareMap[IMU::class.java, IMU_NAME]

                    imu.initialize(
                        IMU.Parameters(
                            RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
                            )
                        )
                    )

                    imu.resetYaw()

                    Globals[mainLooperLens].scheduleCoroutine {
                        when (imuRunMode) {
                            ImuRunMode.DEFAULT -> runDefaultImuHandler(imu)
                            ImuRunMode.THREADED -> runThreadedImuHandler(imuThread.bind(), imu)
                            else -> error("IMU Initialization in RobotOpMode")
                        }
                    }
                }

                Globals[driveLooperLens].let {
                    launch(driveThread.bind()) {
                        it.run()
                    }
                }

                Globals[mainLooperLens].run()
            }
        }
    }
}