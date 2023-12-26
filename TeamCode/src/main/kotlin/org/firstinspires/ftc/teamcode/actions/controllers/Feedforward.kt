package org.firstinspires.ftc.teamcode.actions.controllers

import com.acmerobotics.roadrunner.profile.MotionProfile
import com.acmerobotics.roadrunner.profile.MotionState
import com.outoftheboxrobotics.suspendftc.loopYieldWhile
import com.qualcomm.robotcore.util.ElapsedTime
import kotlinx.coroutines.coroutineScope
import kotlinx.coroutines.launch
import kotlin.math.sign

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
    pidIntegralLimit: Double = Double.MAX_VALUE
) = coroutineScope {
    val timer = ElapsedTime()

    var pidGain = 0.0

    val pidJob = launch { runPidController(
        coefs = pid,
        input = input,
        target = { profile[timer.seconds()].x },
        output = { pidGain = it },
        integralLimit = pidIntegralLimit
    ) }

    loopYieldWhile({ timer.seconds() <= profile.duration() }) {
        output(pidGain + feedforward.computeGain(profile[timer.seconds()]))
    }

    output(feedforward.computeGain(profile.end()))
    pidJob.cancel()
}
