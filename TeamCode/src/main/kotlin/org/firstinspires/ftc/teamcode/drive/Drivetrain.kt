package org.firstinspires.ftc.teamcode.drive

import arrow.core.nonEmptyListOf
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.util.Angle
import com.outoftheboxrobotics.suspendftc.loopYieldWhile
import kotlinx.coroutines.CompletableDeferred
import kotlinx.coroutines.ExperimentalCoroutinesApi
import kotlinx.coroutines.async
import kotlinx.coroutines.channels.Channel
import kotlinx.coroutines.coroutineScope
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.collectLatest
import kotlinx.coroutines.flow.distinctUntilChanged
import kotlinx.coroutines.flow.filter
import kotlinx.coroutines.flow.map
import kotlinx.coroutines.flow.mapLatest
import kotlinx.coroutines.isActive
import kotlinx.coroutines.launch
import org.firstinspires.ftc.teamcode.Globals
import org.firstinspires.ftc.teamcode.RobotState
import org.firstinspires.ftc.teamcode.actions.hardware.setDrivePowers
import org.firstinspires.ftc.teamcode.command.Command
import org.firstinspires.ftc.teamcode.command.Subsystem
import org.firstinspires.ftc.teamcode.commandHandler
import org.firstinspires.ftc.teamcode.imuHandler
import org.firstinspires.ftc.teamcode.next
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence

interface DrivetrainHandler {
    val currentPose: StateFlow<Pose2d>
    val driveState: StateFlow<DriveState>

    fun resetPose(newPose: Pose2d = Pose2d())

    suspend fun followTrajectory(traj: TrajectorySequence)
    fun launchFixpoint(target: Pose2d)
    fun stopCurrentTask()

    suspend fun runHandler()
}

sealed interface DriveState {
    data object Idle : DriveState
    data class Fixpoint(val target: Pose2d) : DriveState
    data class Following(val trajectory: TrajectorySequence) : DriveState
}

class RoadrunnerDrivetrain(private val rrDrive: SampleMecanumDrive) : DrivetrainHandler {
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
            Globals[RobotState.commandHandler]!!
                    .runNewCommand(nonEmptyListOf(Subsystem.DRIVETRAIN), action)
            monitor.complete(Unit)
        }

        monitor.await()
    }

    override suspend fun followTrajectory(traj: TrajectorySequence) = runDriveCommand {
        driveState.value = DriveState.Following(traj)
        rrDrive.followTrajectorySequenceAsync(traj)

        loopYieldWhile({ rrDrive.isBusy }) {
            rrDrive.updateFollower()
        }

        setDrivePowers(0.0, 0.0, 0.0)
    }

    override fun launchFixpoint(target: Pose2d) {
        taskQueue.trySend {
            Globals[RobotState.commandHandler]!!
                .runNewCommand(nonEmptyListOf(Subsystem.DRIVETRAIN)) {
                    driveState.value = DriveState.Fixpoint(target)

                    TODO("Add Fixpoint PID implementation")
                }
        }
    }

    private val stopDrivetrainCommand = Command(nonEmptyListOf(Subsystem.DRIVETRAIN)) {
        driveState.value = DriveState.Idle
    }

    override fun stopCurrentTask() {
        taskQueue.trySend {
            Globals[RobotState.commandHandler]!!.runCommand(stopDrivetrainCommand)
        }
    }

    @OptIn(ExperimentalCoroutinesApi::class)
    private suspend fun runLocalizer() = coroutineScope {
        fun imuAngleAsync() = Globals[RobotState.imuHandler].map {
            async {
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

            rrDrive.updatePoseEstimate()
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
        launch { runLocalizer() }

        launch {
            driveState
                .map { it !is DriveState.Following }
                .distinctUntilChanged()
                .filter { it }
                .collectLatest {
                    rrDrive.breakFollowing()
                }
        }

        while (isActive) {
            val action = taskQueue.receive()

            launch { action.invoke() }
        }
    }
}
