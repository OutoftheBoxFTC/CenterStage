package org.firstinspires.ftc.teamcode.hardware.devices

/**
 * Base interface for optimized device wrappers.
 */
interface KDevice {
    /**
     * Issues a write command to the underlying device.
     */
    fun writeData()

    /**
     * Issues a current read command to the underlying device.
     */
    fun readCurrent(): Double
}
