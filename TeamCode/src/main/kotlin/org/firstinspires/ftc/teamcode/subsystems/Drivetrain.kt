package org.firstinspires.ftc.teamcode.subsystems

import arrow.core.nonEmptyListOf
import arrow.fx.coroutines.resourceScope
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.util.Angle
import com.outoftheboxrobotics.suspendftc.loopYieldWhile
import com.outoftheboxrobotics.suspendftc.suspendUntil
import kotlinx.coroutines.CompletableDeferred
import kotlinx.coroutines.ExperimentalCoroutinesApi
import kotlinx.coroutines.async
import kotlinx.coroutines.channels.Channel
import kotlinx.coroutines.coroutineScope
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.isActive
import kotlinx.coroutines.launch
import kotlinx.coroutines.supervisorScope
import org.firstinspires.ftc.teamcode.Globals
import org.firstinspires.ftc.teamcode.RobotState
import org.firstinspires.ftc.teamcode.actions.controllers.PidCoefs
import org.firstinspires.ftc.teamcode.actions.controllers.runPosePidController
import org.firstinspires.ftc.teamcode.actions.hardware.setDrivePowers
import org.firstinspires.ftc.teamcode.command.Command
import org.firstinspires.ftc.teamcode.command.Subsystem
import org.firstinspires.ftc.teamcode.imuHandler
import org.firstinspires.ftc.teamcode.util.next
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence
import org.firstinspires.ftc.teamcode.util.G

interface DrivetrainHandler {
    val currentPose: StateFlow<Pose2d>
    val driveState: StateFlow<DriveState>

    fun resetPose(newPose: Pose2d = Pose2d())

    suspend fun followTrajectory(traj: TrajectorySequence)
    suspend fun followTrajectoryFixpoint(traj: TrajectorySequence, stopDist: Double = 5.0)
    fun launchFixpoint(target: Pose2d)
    fun stopCurrentTask()

    suspend fun runHandler()
}

sealed interface DriveState {
    data object Idle : DriveState
    data class Fixpoint(val target: Pose2d) : DriveState
    data class Following(val trajectory: TrajectorySequence) : DriveState
}

class RoadrunnerDrivetrain(
    private val getRRDrive: () -> SampleMecanumDrive
) : DrivetrainHandler {
    private lateinit var rrDrive: SampleMecanumDrive

    fun initialize() { rrDrive = getRRDrive() }

    private val translationalPid = PidCoefs(0.5, 0.0, 0.02)
    private val headingPid = PidCoefs(3.2, 0.0, 0.2)

    private var newPose: Pose2d? = null

    override val currentPose = MutableStateFlow(Pose2d())
    override val driveState = MutableStateFlow<DriveState>(DriveState.Idle)

    override fun resetPose(newPose: Pose2d) {
        this.newPose = newPose
    }

    private val taskQueue = Channel<suspend () -> Unit>(capacity = Channel.UNLIMITED)

    private suspend fun runDriveCommand(action: suspend () -> Unit) {
        val monitor = CompletableDeferred<Unit>()

        taskQueue.trySend {
            try {
                Globals.cmd
                    .runNewCommand(nonEmptyListOf(Subsystem.DRIVETRAIN), action)
            } finally {
                monitor.complete(Unit)
            }
        }

        monitor.await()

    }

    override suspend fun followTrajectory(traj: TrajectorySequence) = runDriveCommand {
        resourceScope {
            driveState.value = DriveState.Following(traj)

            install(
                acquire = { rrDrive },
                release = { _, _ ->
                    rrDrive.breakFollowing()
                    rrDrive.setMotorPowers(0.0, 0.0, 0.0, 0.0)
                }
            )

            rrDrive.followTrajectorySequenceAsync(traj)

            suspendUntil { !rrDrive.isBusy }
            driveState.value = DriveState.Idle
        }
    }

    override suspend fun followTrajectoryFixpoint(traj: TrajectorySequence, stopDist: Double) =
        supervisorScope {
            launch { followTrajectory(traj) }
            suspendUntil { currentPose.value.vec().distTo(traj.end().vec()) <= stopDist }
            launchFixpoint(traj.end())
        }

    override fun launchFixpoint(target: Pose2d) {
        taskQueue.trySend {
            Globals.cmd
                .runNewCommand(nonEmptyListOf(Subsystem.DRIVETRAIN)) {
                    driveState.value = DriveState.Fixpoint(target)

                    runPosePidController(
                        translationalCoefs = translationalPid,
                        headingCoefs = headingPid,
                        input = { G.drive.currentPose.value },
                        target = { target },
                        output = { setDrivePowers(it.x, it.y, it.heading) }
                    )
                }
        }
    }

    private val stopDrivetrainCommand = Command(nonEmptyListOf(Subsystem.DRIVETRAIN)) {
        driveState.value = DriveState.Idle
    }

    override fun stopCurrentTask() {
        taskQueue.trySend {
            Globals.cmd.runCommand(stopDrivetrainCommand)
        }
    }

    @OptIn(ExperimentalCoroutinesApi::class)
    private suspend fun runRRDrive() = coroutineScope {
        fun imuAngleAsync() = Globals[RobotState.imuHandler].map {
            async {
                var n = 0

                loopYieldWhile({ n < 100 }) { n++ }

                it.rawAngle.next()
                it.angle
            }
        }.getOrNull()

        var nextImuAngle = imuAngleAsync()
        var lastCorrectedPose = currentPose.value

        loopYieldWhile({ isActive }) {
            newPose?.let {
                rrDrive.poseEstimate = it
                currentPose.value = it
                lastCorrectedPose = it
                newPose = null
                nextImuAngle = imuAngleAsync()
            }

            rrDrive.update()
            currentPose.value = rrDrive.poseEstimate

            nextImuAngle?.let {
                if (it.isCompleted) {
                    val imuAngle = it.getCompleted()

                    val angDelta = Angle.normDelta(imuAngle - currentPose.value.heading)
                    val vecDelta = currentPose.value.vec() - lastCorrectedPose.vec()

                    // Pose estimate will be set in the beginning of the next loop
                    newPose = Pose2d(
                        lastCorrectedPose.vec() + vecDelta.rotated(angDelta),
                        imuAngle
                    )
                }
            }
        }
    }


    override suspend fun runHandler(): Unit = coroutineScope {
        launch { runRRDrive() }

        while (isActive) {
            val action = taskQueue.receive()

            launch { action.invoke() }
        }
    }
}
