package org.firstinspires.ftc.teamcode.actions.hardware

import arrow.optics.optics
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator
import com.acmerobotics.roadrunner.profile.MotionState
import com.outoftheboxrobotics.suspendftc.suspendFor
import com.outoftheboxrobotics.suspendftc.suspendUntil
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.RobotState
import org.firstinspires.ftc.teamcode.outtakeState
import org.firstinspires.ftc.teamcode.util.G

enum class ArmPosition(val pos: Double) {
    FLOOR(1.0),
    OUTTAKE(0.950),
    AUTON_INIT(0.174),
    NEUTRAL(0.036),
    TRANSFER(0.036)
}

@Suppress("unused")
enum class TwistPosition(val pos: Double) {
    HORIZONTAL_0(0.016), // red far
    POS_1(0.241), // red far
    STRAIGHT(0.344),
    POS_2(0.462), // black far
    HORIZONTAL(0.653), // black far
    POS_3(0.853), // black far
}

enum class ClawPosition(val pos: Double, val isBlack: Boolean) {
    BLACK_CLOSE(0.375, true),
    BLACK_OPEN(0.153, true),

    RED_CLOSE(0.829, false),
    RED_OPEN(0.990, false)
}

@optics
data class OuttakeState(
    val twistPos: TwistPosition = TwistPosition.STRAIGHT
) { companion object }

object ArmConfig {
    const val maxArmVel = 3.0
    const val maxArmAccel = 4.0
}

object LiftConfig {
    const val liftDown = -0.6
    const val liftHold = 0.1
    const val liftUp = 1.0

    const val transferHeightMin = 50
}

fun setArmPosition(pos: Double) { G.chub.arm.position = pos }
fun setArmPosition(pos: ArmPosition) = setArmPosition(pos.pos)

fun setTwistPosition(pos: Double) { G.ehub.twist.position = pos }
fun setTwistPosition(pos: TwistPosition) = setTwistPosition(pos.pos).also {
    G[RobotState.outtakeState.twistPos] = pos
}

fun setBlackClawPos(pos: Double) { G.chub.blackClaw.position = pos }
fun setRedClawPos(pos: Double) { G.chub.redClaw.position = pos }
fun setClawPos(pos: ClawPosition) =
    if (pos.isBlack) setBlackClawPos(pos.pos) else setRedClawPos(pos.pos)

fun closeClaws() {
    setClawPos(ClawPosition.RED_CLOSE)
    setClawPos(ClawPosition.BLACK_CLOSE)
}

fun openClaws() {
    setClawPos(ClawPosition.RED_OPEN)
    setClawPos(ClawPosition.BLACK_OPEN)
}

fun liftPos() = G.ehub.outtakeLift.currentPosition

fun liftDown() { G.ehub.outtakeLift.power = LiftConfig.liftDown }
fun liftHold() { G.ehub.outtakeLift.power = LiftConfig.liftHold }
fun liftUp() { G.ehub.outtakeLift.power = LiftConfig.liftUp }

suspend fun liftUpTo(pos: Int) {
    liftUp()
    suspendUntil { liftPos() >= pos }
    liftHold()
}

suspend fun liftDownTo(pos: Int) {
    liftDown()
    suspendUntil { liftPos() <= pos }
    liftHold()
}

suspend fun retractLift() {
    liftDown()
    val timer = ElapsedTime()
    suspendUntil { liftPos() < 3 || timer.seconds() > 0.9 }
    suspendFor(100)
    G.ehub.outtakeLift.power = 0.0
}

/**
 * Moves the arm to the given position using a motion profile.
 */
suspend fun profileArm(targetPos: Double) {
    val profile = MotionProfileGenerator.generateSimpleMotionProfile(
        start = MotionState(G.chub.arm.position, 0.0),
        goal = MotionState(targetPos, 0.0),
        maxVel = ArmConfig.maxArmVel,
        maxAccel = ArmConfig.maxArmAccel
    )

    val timer = ElapsedTime()

    suspendUntil {
        val t = timer.seconds()

        setArmPosition(profile[t].x)
        t >= profile.duration()
    }

    setArmPosition(targetPos)
}

/**
 * Moves the arm to the given position using a motion profile.
 */
suspend fun profileArm(target: ArmPosition) = profileArm(target.pos)
