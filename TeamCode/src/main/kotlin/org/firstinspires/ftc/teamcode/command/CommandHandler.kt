package org.firstinspires.ftc.teamcode.command

import kotlinx.coroutines.Job
import kotlinx.coroutines.coroutineScope
import kotlinx.coroutines.joinAll
import kotlinx.coroutines.launch
import kotlinx.coroutines.sync.Mutex
import kotlinx.coroutines.sync.withLock

class CommandHandler {
    private val lock = Mutex()
    private val currentJobs = mutableMapOf<Subsystem, Job>()

    private suspend fun freeSubsystemsUnsafe(subsystems: List<Subsystem>) {
        currentJobs.filterKeys { it in subsystems }.values.onEach { it.cancel() }.joinAll()
    }

    suspend fun freeSubsystems(subsystems: List<Subsystem>) = lock.withLock {
        freeSubsystemsUnsafe(subsystems)
    }

    suspend fun runCommand(required: List<Subsystem>, command: suspend () -> Unit) = coroutineScope {
        val job: Job

        lock.withLock {
            freeSubsystemsUnsafe(required)

            job = launch { command() }

            required.forEach {
                currentJobs[it] = job
            }
        }

        job.join()
        freeSubsystems(required)
    }
}