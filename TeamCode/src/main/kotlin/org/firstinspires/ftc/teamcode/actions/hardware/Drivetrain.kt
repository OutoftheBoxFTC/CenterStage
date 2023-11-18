package org.firstinspires.ftc.teamcode.actions.hardware

import com.outoftheboxrobotics.suspendftc.loopYieldWhile
import org.firstinspires.ftc.teamcode.RobotState
import org.firstinspires.ftc.teamcode.imuHandler
import org.firstinspires.ftc.teamcode.util.C
import org.firstinspires.ftc.teamcode.util.G
import kotlin.math.cos
import kotlin.math.sin

fun setDrivePowers(x: Double, y: Double, r: Double) = G.chub.run {
    tr.power = +x +y +r
    tl.power = -x +y +r
    bl.power = -x -y +r
    br.power = +x -y +r
}

suspend fun runFieldCentricDrive() {
    val imu = G[RobotState.imuHandler].getOrNull()!!

    loopYieldWhile({ true }) {
        if (C.imuResetAngle) imu.resetAngle()

        val heading = imu.angle

        setDrivePowers(
            C.driveStrafeX * cos(-heading) - C.driveStrafeY * sin(-heading),
            C.driveStrafeX * sin(-heading) + C.driveStrafeY * cos(-heading),
            C.driveTurn
        )
    }
}
