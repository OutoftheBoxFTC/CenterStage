package org.firstinspires.ftc.teamcode

import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.supervisorScope
import org.firstinspires.ftc.teamcode.util.FS

/**
 * State machines where each state is a suspend function that returns the next state.
 *
 * Note that states share the same coroutine scope, so launches in one state are not cancelled when
 * transitioning to the next state.
 */
fun interface FunctionalState {
    /**
     * Run the state and return the next state.
     */
    suspend fun CoroutineScope.runState(): FunctionalState

    /**
     * A special state that indicates that the state machine should stop.
     */
    object StopState : FunctionalState {
        override suspend fun CoroutineScope.runState() = this@StopState
    }
}

/**
 * Run a state machine until [StopState][FunctionalState.StopState] is reached.
 *
 * @param state The initial state.
 */
suspend fun runStateMachine(state: FunctionalState) = supervisorScope {
    var currentState = state

    while (currentState != FunctionalState.StopState) {
        currentState = with(currentState) { runState() }
    }
}

interface StateMachine {
    val defaultState: FS

    suspend fun run() = runStateMachine(defaultState)
}
