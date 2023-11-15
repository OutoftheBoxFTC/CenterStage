package org.firstinspires.ftc.teamcode.actions.hardware

import org.firstinspires.ftc.teamcode.util.G

fun setDrivePowers(x: Double, y: Double, r: Double) = G.chub.run {
    tr.power = +x +y +r
    tl.power = -x +y +r
    bl.power = -x -y +r
    br.power = +x -y +r
}
