package org.firstinspires.ftc.teamcode.statemachine

fun interface FunctionalState {

    suspend fun runState(): FunctionalState

    object StopState : FunctionalState {
        override suspend fun runState() = this
    }
}

suspend fun runStateMachine(state: FunctionalState) {
    var currentState = state

    while (currentState != FunctionalState.StopState) {
        currentState = currentState.runState()
    }
}
