package org.firstinspires.ftc.teamcode.actions.hardware

import org.firstinspires.ftc.teamcode.util.G

enum class IntakeTiltPosition(val pos: Double) {
    TRANSFER_FLAT(0.986),

    TRANSFER(0.079),
    POST_TRANSFER(0.565),

    PRELOAD_HOLD(0.901),

    HIGH(0.655),
    LOW(1.0),

    POS_1(0.838),
    POS_2(0.874),
    POS_3(0.919),
    POS_4(0.967),
}

fun setTiltPosition(pos: Double) { G.chub.intakeTilt.position = pos }
fun setTiltPosition(pos: IntakeTiltPosition) = setTiltPosition(pos.pos)
