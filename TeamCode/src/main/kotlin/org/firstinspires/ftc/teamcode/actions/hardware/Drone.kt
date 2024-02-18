package org.firstinspires.ftc.teamcode.actions.hardware

import org.firstinspires.ftc.teamcode.util.G

enum class DronePosition(val pos: Double) {
    RELEASE(0.202),
    CLOSED(0.904)
}

fun setDronePos(pos: Double) {
    G.ehub.drone.position = pos
}

fun closeDrone() = setDronePos(DronePosition.CLOSED.pos)
fun launchDrone() = setDronePos(DronePosition.RELEASE.pos)