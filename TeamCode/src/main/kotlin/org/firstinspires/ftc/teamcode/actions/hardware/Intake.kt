package org.firstinspires.ftc.teamcode.actions.hardware

import org.firstinspires.ftc.teamcode.util.G

enum class IntakeTiltPosition(val pos: Double) {
    TRANSFER_FLAT(0.767),

    TRANSFER(0.277),
    PRE_TRANSFER(0.503),
    POST_TRANSFER(0.482),

    HIGH(0.639),
    LOW(0.806),

    POS_1(0.708),

    // TODO Finish these positions later
    POS_2(0.742),
    POS_3(0.755),
    POS_4(0.790),
}

fun setTiltPosition(pos: Double) { G.ehub.intakeTilt.position = pos }
fun setTiltPosition(pos: IntakeTiltPosition) = setTiltPosition(pos.pos)
