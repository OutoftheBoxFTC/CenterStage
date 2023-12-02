package org.firstinspires.ftc.teamcode.actions.hardware

import org.firstinspires.ftc.teamcode.util.G

enum class ArmPosition(val pos: Double) {
    FLOOR(1.0),
    OUTTAKE(0.841),
    NEUTRAL(0.266),
    TRANSFER(0.127)
}

enum class TwistPosition(val pos: Double) {
    HORIZONTAL_ISH(0.0),
    POS_1(0.116),
    STRAIGHT(0.245),
    POS_2(0.448),
    HORIZONTAL(0.558),
    POS_3(0.751),
    POS_4(0.982)
}

enum class ClawPosition(val pos: Double, val isBlack: Boolean) {
    BLACK_CLOSE(0.375, true),
    BLACK_OPEN(0.153, true),

    RED_CLOSE(0.829, false),
    RED_OPEN(0.990, false)
}

fun setArmPosition(pos: Double) { G.chub.arm.position = pos }
fun setArmPosition(pos: ArmPosition) = setArmPosition(pos.pos)

fun setTwistPosition(pos: Double) { G.chub.twist.position = pos }
fun setTwistPosition(pos: TwistPosition) = setTwistPosition(pos.pos)

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