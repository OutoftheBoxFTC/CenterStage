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

fun setArmPosition(pos: Double) { G.chub.arm.position = pos }
fun setArmPosition(pos: ArmPosition) = setArmPosition(pos.pos)

fun setTwistPosition(pos: Double) { G.chub.twist.position = pos }
fun setTwistPosition(pos: TwistPosition) = setTwistPosition(pos.pos)