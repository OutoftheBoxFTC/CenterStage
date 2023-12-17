package org.firstinspires.ftc.teamcode.actions.hardware

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.outoftheboxrobotics.suspendftc.suspendFor
import kotlinx.coroutines.cancelAndJoin
import kotlinx.coroutines.coroutineScope
import kotlinx.coroutines.launch
import org.firstinspires.ftc.teamcode.RobotState
import org.firstinspires.ftc.teamcode.driveState
import org.firstinspires.ftc.teamcode.util.G
import org.firstinspires.ftc.teamcode.util.mainLoop

fun resetDrivePose(newPose: Pose2d = Pose2d()) {
    resetImuAngle(newPose.heading)
    G[RobotState.driveState.currentPose] = newPose
}

suspend fun intakeTransfer() {
    G.ehub.intakeRoller.power = -0.8
    setTiltPosition(IntakeTiltPosition.HIGH)
    retractExtension()
    suspendFor(1000)
    profileArm(ArmPosition.TRANSFER)
    G.ehub.intakeRoller.power = 0.0
    closeClaws()
    suspendFor(100)
    setArmPosition(ArmPosition.NEUTRAL)
    setTiltPosition(IntakeTiltPosition.LOW)
    suspendFor(200)
    setTiltPosition(IntakeTiltPosition.HIGH)
}

suspend fun swoop(
    start: Vector2d,
    target: Vector2d,
    heading: Double,
    extensionLength: Int?
) = coroutineScope {
    val driveJob = launch {
        var driveMultiplier = 1.0

        val currentMonitor = launch {
            mainLoop {
                G.ehub.readCurrents()
                G.chub.readCurrents()

                val driveCurrentBudget = 19.8 - G.ehub.hubCurrent
                val driveCurrent = G.chub.hubCurrent

                driveMultiplier =
                    (driveMultiplier * driveCurrentBudget / driveCurrent)
                        .coerceIn(0.0..1.0)

                suspendFor(200)
            }
        }

        followLinePath(
            start,
            target,
            heading,
            maxPower = { driveMultiplier }
        )

        launchFixpoint(Pose2d(target, heading))
        currentMonitor.cancelAndJoin()
    }

    if (extensionLength != null) runExtensionTo(extensionLength, keepPid = true)
    else {
        intakeTransfer()
        profileArm(ArmPosition.OUTTAKE)
    }

    driveJob.join()
}
