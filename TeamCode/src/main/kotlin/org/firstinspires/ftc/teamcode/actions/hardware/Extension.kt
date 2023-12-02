package org.firstinspires.ftc.teamcode.actions.hardware

import arrow.core.nel
import arrow.optics.optics
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator
import com.acmerobotics.roadrunner.profile.MotionState
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.RobotState
import org.firstinspires.ftc.teamcode.actions.controllers.FeedforwardCoefs
import org.firstinspires.ftc.teamcode.actions.controllers.PidCoefs
import org.firstinspires.ftc.teamcode.actions.controllers.runMotionProfile
import org.firstinspires.ftc.teamcode.actions.controllers.runPidController
import org.firstinspires.ftc.teamcode.Subsystem
import org.firstinspires.ftc.teamcode.extensionState
import org.firstinspires.ftc.teamcode.mainLooper
import org.firstinspires.ftc.teamcode.util.G

@optics
data class ExtensionState(
    val currentMotionState: MotionState
) { companion object }

object ExtensionConfig {
    const val maxVel = 0.0
    const val maxAccel = 0.0

    val feedforwardCoefs: FeedforwardCoefs = FeedforwardCoefs(0.0, 0.0, 0.0, 0.0)
    val pidCoefs: PidCoefs = PidCoefs(0.0, 0.0, 0.0)
}

fun currentExtensionMotionState() = G[RobotState.extensionState.currentMotionState]
fun currentExtensionLength() = currentExtensionMotionState().x
fun setExtensionPower(power: Double) { G.ehub.extension.power = power }
fun setExtensionMotionState(newState: MotionState) {
    G[RobotState.extensionState.currentMotionState] = newState
}
fun updateExtensionMotionState() = setExtensionMotionState(MotionState(currentExtensionLength(), 0.0))

suspend fun profileTo(target: MotionState) = G.cmd.runNewCommand(Subsystem.EXTENSION.nel()) {
    val timer = ElapsedTime()
    val profile = MotionProfileGenerator.generateSimpleMotionProfile(
        currentExtensionMotionState(), target, ExtensionConfig.maxVel, ExtensionConfig.maxAccel
    )

    runMotionProfile(
        profile = profile,
        feedforward = ExtensionConfig.feedforwardCoefs,
        pid = ExtensionConfig.pidCoefs,
        input = ::currentExtensionLength,
        output = {
            setExtensionPower(it)
            G[RobotState.extensionState.currentMotionState] = profile[timer.seconds()]
        }
    )
}

fun launchExtensionPid(target: Double) = G[RobotState.mainLooper].scheduleCoroutine {
    G.cmd.runNewCommand(Subsystem.EXTENSION.nel()) {
        runPidController(
            coefs = ExtensionConfig.pidCoefs,
            input = ::currentExtensionLength,
            target = { target },
            output = ::setExtensionPower
        )
    }
}
