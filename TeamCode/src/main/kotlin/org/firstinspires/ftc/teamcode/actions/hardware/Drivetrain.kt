package org.firstinspires.ftc.teamcode.actions.hardware

import com.outoftheboxrobotics.suspendftc.loopYieldWhile
import org.firstinspires.ftc.teamcode.RobotState
import org.firstinspires.ftc.teamcode.imuState
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive
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

fun setAdjustedDrivePowers(x: Double, y: Double, r: Double) {
    val multiplier = 12.0 / G.chub.voltageSensor.voltage

    setDrivePowers(
        multiplier * x,
        multiplier * y * SampleMecanumDrive.LATERAL_MULTIPLIER,
        multiplier * r
    )
}

suspend fun runFieldCentricDrive(): Nothing {
    loopYieldWhile({ true }) {
        if (C.imuResetAngle) resetImuAngle()

        val heading = G[RobotState.imuState].angle

        setDrivePowers(
            C.driveStrafeX * cos(-heading) - C.driveStrafeY * sin(-heading),
            C.driveStrafeX * sin(-heading) + C.driveStrafeY * cos(-heading),
            C.driveTurn
        )
    }

    error("Return from runFieldCentricDrive")
}
