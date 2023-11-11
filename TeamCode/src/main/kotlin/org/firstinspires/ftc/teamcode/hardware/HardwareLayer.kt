package org.firstinspires.ftc.teamcode.hardware

import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareDevice
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.Globals
import org.firstinspires.ftc.teamcode.ReadOnlyProperty
import org.firstinspires.ftc.teamcode.ReadWriteProperty

abstract class HardwareLayer(protected val hwMap: HardwareMap, hubName: String) {
    private val callbacks = mutableListOf<() -> Unit>()

    private val hub = hwMap[LynxModule::class.java, hubName]

    init {
        hub.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL
    }

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

    protected fun encoder(name: String, reversed: Boolean = false) =
        inputDevice<DcMotorEx, _>(name, config = {}, 0) {
            currentPosition.let { if (reversed) -it else it }
        }

    protected fun motor(name: String): KMotor {
        val motor = hwMap[DcMotorEx::class.java, name]
        return KMotor().also {
            callbacks.add { it.applyChanges(motor) }
        }
    }

    protected fun servo(name: String): KServo {
        val servo = hwMap.servo[name]
        return KServo().also {
            callbacks.add { it.applyChanges(servo) }
        }
    }

    private var started = false
    private val timer = ElapsedTime()
    private val sublog = Globals.log.hardware.sublog(hubName)

    fun syncHardware() {
        hub.clearBulkCache()
        callbacks.forEach { it.invoke() }

        if (started) sublog["looptime (ms)"] = timer.milliseconds()
        else {
            started = true
            timer.reset()
        }

        sublog.collect()
    }
}


