package org.firstinspires.ftc.teamcode.actions.controllers

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.util.Angle
import com.outoftheboxrobotics.suspendftc.yieldLooper
import com.qualcomm.robotcore.util.ElapsedTime
import kotlinx.coroutines.coroutineScope
import kotlinx.coroutines.launch
import org.firstinspires.ftc.teamcode.util.mainLoop
import kotlin.math.min

data class PidCoefs(
    var kP: Double,
    var kI: Double,
    var kD: Double
) {
    fun computeGain(error: Double, integral: Double, deriv: Double) =
        kP*error + kI*integral + kD*deriv
}

/**
 * Runs a PID Controller loop indefinitely.
 *
 * @param coefs The PID coefficients.
 * @param input The input function.
 * @param target The target function.
 * @param output The output function.
 * @param integralLimit The maximum value of the integral term.
 */
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

    mainLoop {
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

/**
 * Runs a PID Pose lock controller loop indefinitely.
 *
 * @param translationalCoefs The translational PID coefficients.
 * @param headingCoefs The heading PID coefficients.
 * @param input The input function.
 * @param target The target function.
 * @param output The output function.
 */
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

    // Component PID controllers
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
        mainLoop {
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

    mainLoop {
        output(
            Pose2d(
                // Rotate to account for heading error - always drives towards target
                Vector2d(x, y).rotated(-input().heading),
                r
            )
        )
    }
}
