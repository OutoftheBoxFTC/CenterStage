package org.firstinspires.ftc.teamcode.actions.controllers

import com.acmerobotics.roadrunner.profile.MotionProfile
import com.acmerobotics.roadrunner.profile.MotionState
import com.outoftheboxrobotics.suspendftc.loopYieldWhile
import com.outoftheboxrobotics.suspendftc.yieldLooper
import com.qualcomm.robotcore.util.ElapsedTime
import kotlinx.coroutines.coroutineScope
import kotlinx.coroutines.launch
import org.firstinspires.ftc.teamcode.util.mainLoop
import kotlin.math.abs
import kotlin.math.sign
import kotlin.math.sqrt

data class FeedforwardCoefs(
    var kG: Double,
    var kS: Double,
    var kV: Double,
    var kA: Double
) {
    fun computeGain(state: MotionState) =
        kG + kS*sign(state.v) + kV*state.v + kA*state.a
}

/**
 * Runs a motion profile with feedforward and PID control.
 *
 * @param profile The motion profile.
 * @param feedforward The feedforward coefficients.
 * @param pid The PID coefficients.
 * @param input The input function.
 * @param output The output function.
 * @param pidIntegralLimit The maximum value of the integral term.
 */
suspend inline fun runMotionProfile(
    profile: MotionProfile,
    feedforward: FeedforwardCoefs,
    pid: PidCoefs,
    crossinline input: () -> Double,
    crossinline output: (Double) -> Unit,
    pidIntegralLimit: Double = Double.MAX_VALUE,
    hz: Int? = null
) = coroutineScope {
    val timer = ElapsedTime()

    var pidGain = 0.0

    val pidJob = launch { runPidController(
        coefs = pid,
        input = input,
        target = { profile[timer.seconds()].x },
        output = { pidGain = it },
        integralLimit = pidIntegralLimit,
        hz = hz
    ) }

    loopYieldWhile({ timer.seconds() <= profile.duration() }) {
        output(pidGain + feedforward.computeGain(profile[timer.seconds()]))
    }

    output(feedforward.computeGain(profile.end()))
    pidJob.cancel()
}

suspend inline fun runVeloPid(
    feedforward: FeedforwardCoefs,
    pid: PidCoefs,
    crossinline input: () -> Double,
    crossinline target: () -> Double,
    crossinline output: (Double) -> Unit,
    maxVel: Double,
    maxAccel: Double,
    pidIntegralLimit: Double = Double.MAX_VALUE,
    hz: Int? = null,

    // For testing
    crossinline targetMotionStateDebugging: (MotionState) -> Unit = {}
): Nothing = coroutineScope {
    val timer = ElapsedTime()

    var lastInput = input()
    var lastTarget = target()

    var targetVel = 0.0
    var pidGain = 0.0

    var resetIntegral = false

    yieldLooper()

    var currentVel = (input() - lastInput) / timer.seconds()

    launch {
        runPidController(
            coefs = pid,
            input = { currentVel },
            target = { targetVel },
            output = { pidGain = it },
            integralLimit = pidIntegralLimit,
            hz = hz,
            resetIntegral = {
                resetIntegral.also {
                    if (it) resetIntegral = false
                }
            }
        )
    }

    timer.reset()

    mainLoop {
        val dt = timer.seconds()
        timer.reset()

        val target = target()
        val input = input()

        currentVel = (input - lastInput) / dt
        lastInput = input

        if (target != lastTarget) {
            resetIntegral = true
            lastTarget = target
        }

        val disp = target - input
        val dir = sign(disp)

        val targetState = listOf(
            // Max Vel
            MotionState(input, maxVel, 0.0),

            // Accel limited
            MotionState(input, dir * targetVel + maxAccel * dt, maxAccel),

            // Decel limited
            MotionState(input, sqrt(2*maxAccel*abs(disp)), -maxAccel)
        )
            .minBy { it.v }
            .let { MotionState(it.x, it.v * dir, it.a * dir) }

        targetVel = targetState.v

        output(feedforward.computeGain(targetState) + pidGain)
        targetMotionStateDebugging(targetState)
    }
}
