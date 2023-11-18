package org.firstinspires.ftc.teamcode.subsystems

import arrow.core.nonEmptyListOf
import arrow.fx.coroutines.resourceScope
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
import kotlinx.coroutines.isActive
import kotlinx.coroutines.launch
import org.firstinspires.ftc.teamcode.Globals
import org.firstinspires.ftc.teamcode.RobotState
import org.firstinspires.ftc.teamcode.command.Command
import org.firstinspires.ftc.teamcode.command.Subsystem
import org.firstinspires.ftc.teamcode.imuHandler
import org.firstinspires.ftc.teamcode.util.next
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
            Globals.cmd
                    .runNewCommand(nonEmptyListOf(Subsystem.DRIVETRAIN), action)
            monitor.complete(Unit)
        }

        monitor.await()
    }

    override suspend fun followTrajectory(traj: TrajectorySequence) = runDriveCommand {
        resourceScope {
            driveState.value = DriveState.Following(traj)

            install(
                acquire = {
                    Globals.chub.enableDriveMotors = false
                    rrDrive
                },
                release = { _, _ ->
                    rrDrive.breakFollowing()
                    rrDrive.setMotorPowers(0.0, 0.0, 0.0, 0.0)
                    Globals.chub.enableDriveMotors = true
                }
            )

            rrDrive.followTrajectorySequenceAsync(traj)

            loopYieldWhile({ rrDrive.isBusy }) {
                rrDrive.updateFollower()
            }
        }
    }

    override fun launchFixpoint(target: Pose2d) {
        taskQueue.trySend {
            Globals.cmd
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
            Globals.cmd.runCommand(stopDrivetrainCommand)
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

        while (isActive) {
            val action = taskQueue.receive()

            launch { action.invoke() }
        }
    }
}
