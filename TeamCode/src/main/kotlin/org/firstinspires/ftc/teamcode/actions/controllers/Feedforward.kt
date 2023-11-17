package org.firstinspires.ftc.teamcode.actions.controllers

import com.acmerobotics.roadrunner.profile.MotionProfile
import com.acmerobotics.roadrunner.profile.MotionState
import com.outoftheboxrobotics.suspendftc.loopYieldWhile
import com.qualcomm.robotcore.util.ElapsedTime
import kotlinx.coroutines.coroutineScope
import kotlinx.coroutines.launch
import kotlin.math.sign

data class FeedforwardCoefs(
    val kG: Double,
    val kS: Double,
    val kV: Double,
    val kA: Double
) {
    fun computeGain(state: MotionState) =
        kG + kS*sign(state.v) + kV*state.v + kA*state.a
}

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
