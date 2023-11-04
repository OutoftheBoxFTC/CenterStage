package org.firstinspires.ftc.teamcode.command

import arrow.core.Nel
import arrow.fx.stm.TMap
import arrow.fx.stm.atomically
import arrow.fx.stm.check
import kotlinx.coroutines.CoroutineStart
import kotlinx.coroutines.Job
import kotlinx.coroutines.coroutineScope
import kotlinx.coroutines.joinAll
import kotlinx.coroutines.launch

class Command(val required: Nel<Subsystem>, val action: suspend () -> Unit)

private class RunningCommand(val job: Job, val command: Command)

class CommandHandler private constructor(
        private val starting: TMap<Subsystem, RunningCommand>,
        private val active: TMap<Subsystem, RunningCommand>,
        private val cancelling: TMap<Subsystem, RunningCommand>
) {
    suspend fun runCommand(command: Command) = coroutineScope {
        val required = command.required
        val runningCommand = RunningCommand(
            launch(start = CoroutineStart.LAZY) { command.action() },
            command
        )

        val cancelledJobs = atomically {
            if (required.any { active[it]?.command == command }) return@atomically null

            check(required.all { cancelling[it] == null && starting[it] == null })

            required.mapNotNull {
                val cmd = active[it]

                if (cmd != null) {
                    active.remove(it)
                    cancelling[it] = cmd

                    cmd.job
                } else {
                    starting[it] = runningCommand

                    null
                }
            }
        } ?: return@coroutineScope

        cancelledJobs.onEach { it.cancel() }.joinAll()

        atomically {
            required.filter { cancelling[it] != null }.forEach {
                cancelling.remove(it)
            }

            required.filter { starting[it] != null }.forEach {
                starting.remove(it)
            }

            required.forEach {
                active[it] = runningCommand
            }
        }

        runningCommand.job.join()
    }

    suspend fun runNewCommand(required: Nel<Subsystem>, action: suspend () -> Unit) =
        runCommand(Command(required, action))

    companion object {
        suspend fun new() = CommandHandler(TMap.new(), TMap.new(), TMap.new())
    }
}