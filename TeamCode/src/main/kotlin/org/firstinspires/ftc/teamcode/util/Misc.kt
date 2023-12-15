@file:Suppress("unused")

package org.firstinspires.ftc.teamcode.util

import com.acmerobotics.roadrunner.geometry.Pose2d
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.Controls
import org.firstinspires.ftc.teamcode.FunctionalState
import org.firstinspires.ftc.teamcode.Globals
import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceBuilder
import kotlin.properties.ReadOnlyProperty
import kotlin.properties.ReadWriteProperty
import kotlin.reflect.KProperty

typealias G = Globals
typealias C = Controls
typealias FS = FunctionalState

interface ReadOnlyProperty<out V> : ReadOnlyProperty<Any?, V> {
    val value: V

    override fun getValue(thisRef: Any?, property: KProperty<*>) = value
}

interface ReadWriteProperty<V> : ReadWriteProperty<Any?, V> {
    var value: V

    override fun getValue(thisRef: Any?, property: KProperty<*>) = value
    override fun setValue(thisRef: Any?, property: KProperty<*>, value: V) { this.value = value }
}

operator fun Telemetry.set(caption: String, value: Any) { addData(caption, value) }

fun buildTrajectory(
    startPose: Pose2d,
    block: TrajectorySequenceBuilder.() -> Unit
): TrajectorySequence = SampleMecanumDrive
    .trajectorySequenceBuilder(startPose)
    .apply(block)
    .build()

fun TrajectorySequenceBuilder.setAccelConstraint(maxAccel: Double): TrajectorySequenceBuilder =
    setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(maxAccel))

fun TrajectorySequenceBuilder.setVelConstraint(maxVel: Double): TrajectorySequenceBuilder =
    setVelConstraint(
        SampleMecanumDrive.getVelocityConstraint(
            maxVel,
            DriveConstants.MAX_ANG_VEL,
            DriveConstants.TRACK_WIDTH
        )
    )
