package org.firstinspires.ftc.teamcode

import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.supervisorScope

fun interface FunctionalState {

    suspend fun CoroutineScope.runState(): FunctionalState

    object StopState : FunctionalState {
        override suspend fun CoroutineScope.runState() = this@StopState
    }
}

suspend fun runStateMachine(state: FunctionalState) = supervisorScope {
    var currentState = state

    while (currentState != FunctionalState.StopState) {
        currentState = with(currentState) { runState() }
    }
}
