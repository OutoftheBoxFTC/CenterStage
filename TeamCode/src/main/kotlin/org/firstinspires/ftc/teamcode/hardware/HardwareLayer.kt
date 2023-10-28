package org.firstinspires.ftc.teamcode.hardware

import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareDevice
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.ReadOnlyProperty
import org.firstinspires.ftc.teamcode.ReadWriteProperty

abstract class HardwareLayer(protected val hwMap: HardwareMap) {
    private val callbacks = mutableListOf<() -> Unit>()

    protected fun <T> inputField(default: T, getInput: () -> T) = object : ReadOnlyProperty<T> {
        override var value = default

        init {
            callbacks.add { value = getInput() }
        }
    }

    protected fun <T> outputField(default: T, setOutput: (T) -> Unit) = object :
        ReadWriteProperty<T> {
        override var value = default

        init {
            callbacks.add { setOutput(value) }
        }
    }

    protected inline fun <reified T : HardwareDevice, V> inputDevice(
        name: String,
        config: T.() -> Unit,
        default: V,
        crossinline getInput: T.() -> V
    ): ReadOnlyProperty<V> {
        val device = hwMap.get(T::class.java, name).also(config)
        return inputField(default) { device.getInput() }
    }

    protected inline fun <reified T : HardwareDevice, V> outputDevice(
        name: String,
        config: T.() -> Unit,
        default: V,
        crossinline setOutput: T.(V) -> Unit
    ): ReadWriteProperty<V> {
        val device = hwMap.get(T::class.java, name).also(config)
        return outputField(default) { device.setOutput(it) }
    }

    protected fun motor(name: String, config: DcMotorEx.() -> Unit) =
        outputDevice(name, config, 0.0) { power = it }

    protected fun servo(name: String, config: Servo.() -> Unit) =
        outputDevice(name, config, 0.0) { position = it }

    protected fun encoder(name: String, reversed: Boolean = false) =
        inputDevice<DcMotorEx, _>(name, config = {}, 0) {
            currentPosition.let { if (reversed) -it else it }
        }

    fun syncHardware() {
        callbacks.forEach { it.invoke() }
    }
}


