package org.firstinspires.ftc.teamcode.actions.hardware

import org.firstinspires.ftc.teamcode.util.G

enum class IntakeTiltPosition(val pos: Double) {
    TRANSFER(0.215),

    HIGH(0.450),

    POS_1(0.689),
    POS_2(0.742),
    POS_3(0.755),
    POS_4(0.790),

    LOW(0.795)
}

fun setTiltPosition(pos: Double) { G.ehub.intakeTilt.position = pos }
fun setTiltPosition(pos: IntakeTiltPosition) = setTiltPosition(pos.pos)
