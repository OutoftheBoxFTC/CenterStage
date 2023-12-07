package org.firstinspires.ftc.teamcode.opmodes

import android.os.Build
import androidx.annotation.RequiresApi
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.outoftheboxrobotics.suspendftc.loopYieldWhile
import com.outoftheboxrobotics.suspendftc.suspendUntil
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.cancelAndJoin
import kotlinx.coroutines.coroutineScope
import kotlinx.coroutines.launch
import kotlinx.coroutines.withContext
import kotlinx.datetime.Clock
import kotlinx.datetime.Instant
import kotlinx.serialization.Serializable
import kotlinx.serialization.encodeToString
import kotlinx.serialization.json.Json
import org.firstinspires.ftc.robotcore.internal.system.AppUtil
import org.firstinspires.ftc.teamcode.actions.hardware.currentDrivePose
import org.firstinspires.ftc.teamcode.actions.hardware.launchFixpoint
import org.firstinspires.ftc.teamcode.actions.hardware.resetDrivePose
import org.firstinspires.ftc.teamcode.actions.hardware.runFieldCentricDrive
import org.firstinspires.ftc.teamcode.actions.hardware.setDrivePowers
import org.firstinspires.ftc.teamcode.actions.hardware.setDrivetrainIdle
import org.firstinspires.ftc.teamcode.util.mainLoop
import org.firstinspires.ftc.teamcode.util.set
import kotlin.io.path.createParentDirectories
import kotlin.io.path.deleteIfExists
import kotlin.io.path.exists
import kotlin.io.path.readText
import kotlin.io.path.writeText

@TeleOp
class FastloadOpmode : RobotOpMode(
    monitorOpmodeStop = false
) {
    private val startingPose = Pose2d()

    private suspend fun runAction() {
        // Code here

    }

    @RequiresApi(Build.VERSION_CODES.O)
    private fun poseStoragePath() = AppUtil.ROOT_FOLDER
        .toPath()
        .resolve("teamcode/pose_storage.json")
        .createParentDirectories()

    @RequiresApi(Build.VERSION_CODES.O)
    override suspend fun runSuspendOpMode() = coroutineScope {
        var loadedStoredPose = false

        withContext(Dispatchers.IO) {
            runCatching {
                val storagePath = poseStoragePath()

                if (storagePath.exists()) {
                    Json
                        .decodeFromString<StoredPose>(storagePath.readText())
                        .takeIf { (Clock.System.now() - it.date).inWholeMinutes < 5.0 }
                } else null
            }.getOrNull()
        }?.let {
            resetDrivePose(Pose2d(it.x, it.y, it.heading))
            loadedStoredPose = true
        }

        launch {
            loopYieldWhile({ opModeInInit() }) {
                telemetry["Pose Storage Status"] = if (loadedStoredPose) "LOADED" else "DEFAULT"
                if (gamepad1.a) {
                    resetDrivePose()
                    loadedStoredPose = false
                }
            }
        }

        launch {
            var wasActive = true

            suspendUntil {
                if (isStopRequested) true
                else {
                    wasActive = opModeIsActive()
                    false
                }
            }

            withContext(Dispatchers.IO) {
                if (wasActive) {
                    val pose = currentDrivePose()

                    poseStoragePath().writeText(
                        Json.encodeToString(
                            StoredPose(Clock.System.now(), pose.x, pose.y, pose.heading)
                        )
                    )
                } else {
                    poseStoragePath().deleteIfExists()
                }
            }

            manualStop = true
        }

        suspendUntilStart()

        var status = "IDLE"

        launch { mainLoop { telemetry["Status"] = status } }

        while (true) {
            val driveJob = launch { runFieldCentricDrive() }
            suspendUntil { gamepad1.x }
            driveJob.cancelAndJoin()

            status = "FIXPOINT"
            launchFixpoint(startingPose)
            suspendUntil { gamepad1.y }

            status = "ACTIVE"
            setDrivetrainIdle()
            val job = launch { runAction() }

            suspendUntil { job.isCompleted || gamepad1.b }
            job.cancelAndJoin()

            status = "IDLE"
            setDrivetrainIdle()
            setDrivePowers(0.0, 0.0, 0.0)
        }
    }
}

@Serializable
data class StoredPose(
    val date: Instant,
    val x: Double,
    val y: Double,
    val heading: Double
)
