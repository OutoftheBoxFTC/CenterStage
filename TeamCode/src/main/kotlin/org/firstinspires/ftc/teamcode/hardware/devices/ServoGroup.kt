package org.firstinspires.ftc.teamcode.hardware.devices

import arrow.core.Nel
import com.qualcomm.robotcore.hardware.HardwareDevice
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.ServoController

class ServoGroup(private val servos: Nel<Servo>) : Servo {
    private var direction = Servo.Direction.FORWARD

    override fun getManufacturer(): HardwareDevice.Manufacturer = servos.head.manufacturer

    override fun getDeviceName() = servos.map { it.deviceName }.joinToString()

    override fun getConnectionInfo() = servos.map { it.connectionInfo }.joinToString()

    override fun getVersion() = servos.head.version

    override fun resetDeviceConfigurationForOpMode() = servos.forEach { it.resetDeviceConfigurationForOpMode() }

    override fun close() = servos.forEach { it.close() }

    override fun getController(): ServoController = servos.head.controller

    override fun getPortNumber() = servos.head.portNumber

    override fun setDirection(direction: Servo.Direction) { this.direction = direction }

    override fun getDirection() = direction

    override fun setPosition(position: Double) = servos.forEach { it.position = position }

    override fun getPosition() = servos.head.position

    override fun scaleRange(min: Double, max: Double) = servos.forEach { it.scaleRange(min, max) }
}