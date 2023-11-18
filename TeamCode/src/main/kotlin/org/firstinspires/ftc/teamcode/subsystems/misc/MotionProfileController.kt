package org.firstinspires.ftc.teamcode.subsystems.misc

import arrow.core.nel
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator
import com.acmerobotics.roadrunner.profile.MotionState
import org.firstinspires.ftc.teamcode.Globals
import org.firstinspires.ftc.teamcode.actions.controllers.FeedforwardCoefs
import org.firstinspires.ftc.teamcode.actions.controllers.PidCoefs
import org.firstinspires.ftc.teamcode.actions.controllers.runMotionProfile
import org.firstinspires.ftc.teamcode.command.Subsystem

abstract class MotionProfileController(
    private val feedforwardCoefs: FeedforwardCoefs,
    private val pidCoefs: PidCoefs,
    private val maxVel: Double,
    private val maxAccel: Double,
    private val subsystem: Subsystem,
    initialState: MotionState
) {
    var currentMotionState = initialState

    abstract fun feedback(): Double
    abstract fun updateOutput(output: Double)

    suspend fun profileTo(target: MotionState) = Globals.cmd.runNewCommand(subsystem.nel()) {
        runMotionProfile(
            profile = MotionProfileGenerator.generateSimpleMotionProfile(
                currentMotionState, target, maxVel, maxAccel
            ),
            feedforward = feedforwardCoefs,
            pid = pidCoefs,
            input = ::feedback,
            output = ::updateOutput
        )
    }
}