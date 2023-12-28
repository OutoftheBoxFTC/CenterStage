package org.firstinspires.ftc.teamcode.actions.hardware

import org.firstinspires.ftc.teamcode.util.G

enum class IntakeTiltPosition(val pos: Double) {
    HIGH(0.688),

    POS_1(0.756),
    POS_2(0.816),
    POS_3(0.861),
    POS_4(0.913),

    LOW(0.969)
}

fun setTiltPosition(pos: Double) { G.ehub.intakeTilt.position = pos }
fun setTiltPosition(pos: IntakeTiltPosition) = setTiltPosition(pos.pos)
