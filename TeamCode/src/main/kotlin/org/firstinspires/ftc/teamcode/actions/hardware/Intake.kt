package org.firstinspires.ftc.teamcode.actions.hardware

import org.firstinspires.ftc.teamcode.util.G

enum class IntakeTiltPosition(val pos: Double) {
    HIGH(0.144),

    POS_1(0.238),
    POS_2(0.286),
    POS_3(0.347),
    POS_4(0.406),

    LOW(0.446)
}

fun setTiltPosition(pos: Double) { G.ehub.intakeTilt.position = pos }
fun setTiltPosition(pos: IntakeTiltPosition) = setTiltPosition(pos.pos)
