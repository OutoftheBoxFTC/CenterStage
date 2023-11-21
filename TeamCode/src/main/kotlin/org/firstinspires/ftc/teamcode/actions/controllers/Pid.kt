package org.firstinspires.ftc.teamcode.actions.controllers

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.util.Angle
import com.outoftheboxrobotics.suspendftc.loopYieldWhile
import com.outoftheboxrobotics.suspendftc.yieldLooper
import com.qualcomm.robotcore.util.ElapsedTime
import kotlinx.coroutines.coroutineScope
import kotlinx.coroutines.launch
import kotlin.math.min

data class PidCoefs(
    var kP: Double,
    var kI: Double,
    var kD: Double
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
): Nothing {
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

    error("Return from runPidController")
}

suspend inline fun runPosePidController(
    translationalCoefs: PidCoefs,
    headingCoefs: PidCoefs,
    crossinline input: () -> Pose2d,
    crossinline target: () -> Pose2d,
    crossinline output: (Pose2d) -> Unit
): Nothing = coroutineScope {
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

    launch {
        val timer = ElapsedTime()
        var lastError = target().heading - input().heading
        var errorAcc = 0.0

        r = headingCoefs.kP * lastError

        yieldLooper()
        loopYieldWhile({ true }) {
            val dt = timer.seconds()
            timer.reset()

            val error = Angle.normDelta(target().heading - input().heading)
            val ddt = (error - lastError) / dt

            lastError = error
            errorAcc += error * dt
            errorAcc = min(errorAcc, Double.MAX_VALUE)

            headingCoefs.computeGain(error, errorAcc, ddt)
            r = headingCoefs.computeGain(error, errorAcc, ddt)
        }
    }

    loopYieldWhile({ true }) {
        output(
            Pose2d(
                Vector2d(x, y).rotated(-input().heading),
                r
            )
        )
    }

    error("Return from runPosePidController")
}
