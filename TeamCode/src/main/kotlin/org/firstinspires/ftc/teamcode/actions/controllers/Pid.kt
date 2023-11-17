package org.firstinspires.ftc.teamcode.actions.controllers

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.outoftheboxrobotics.suspendftc.loopYieldWhile
import com.outoftheboxrobotics.suspendftc.yieldLooper
import com.qualcomm.robotcore.util.ElapsedTime
import kotlinx.coroutines.coroutineScope
import kotlinx.coroutines.launch
import kotlin.math.min

data class PidCoefs(
    val kP: Double,
    val kI: Double,
    val kD: Double
) {
    fun computeGain(error: Double, integral: Double, deriv: Double) =
        kP*error + kI*integral + kD*deriv
}

suspend inline fun runPidController(
    coefs: PidCoefs,
    input: () -> Double,
    target: () -> Double,
    output: (Double) -> Unit,
    integralLimit: Double = Double.MAX_VALUE
) {
    val timer = ElapsedTime()

    var lastError = target() - input()
    var errorAcc = 0.0

    output(coefs.kP * lastError)

    yieldLooper()

    loopYieldWhile({ true }) {
        val dt = timer.seconds()
        timer.reset()

        val error = target() - input()
        val ddt = (error - lastError) / dt

        lastError = error
        errorAcc += error*dt
        errorAcc = min(errorAcc, integralLimit)

        output(coefs.computeGain(error, errorAcc, ddt))
    }
}

suspend inline fun runPosePidController(
    translationalCoefs: PidCoefs,
    headingCoefs: PidCoefs,
    crossinline input: () -> Pose2d,
    crossinline target: () -> Pose2d,
    crossinline output: (Pose2d) -> Unit
) = coroutineScope {
    var x = 0.0
    var y = 0.0
    var r = 0.0

    launch { runPidController(
        coefs = translationalCoefs,
        input = { input().x },
        target = { target().x },
        output = { x = it }
    ) }

    launch { runPidController(
        coefs = translationalCoefs,
        input = { input().y },
        target = { target().y },
        output = { y = it }
    ) }

    launch { runPidController(
        coefs = headingCoefs,
        input = { input().heading },
        target = { target().heading },
        output = { r = it }
    ) }

    loopYieldWhile({ true }) {
        output(
            Pose2d(
                Vector2d(x, y).rotated(-input().heading),
                r
            )
        )
    }
}
