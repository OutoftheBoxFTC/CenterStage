package org.firstinspires.ftc.teamcode.hardware

import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import org.firstinspires.ftc.teamcode.Globals
import org.firstinspires.ftc.teamcode.hardware.devices.KCRServo
import org.firstinspires.ftc.teamcode.util.ReadOnlyProperty
import org.firstinspires.ftc.teamcode.util.ReadWriteProperty
import org.firstinspires.ftc.teamcode.hardware.devices.KDevice
import org.firstinspires.ftc.teamcode.hardware.devices.KMotor
import org.firstinspires.ftc.teamcode.hardware.devices.KServo
import kotlin.math.roundToInt

abstract class HardwareLayer(protected val hwMap: HardwareMap, private val hubName: String) {
    private val callbacks = mutableListOf<() -> Unit>()
    private val currentReadCallbacks = mutableListOf<() -> Unit>()

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

    protected fun <T : KDevice> T.registerDevice() = apply {
        callbacks.add(this::writeData)
        currentReadCallbacks.add(this::readCurrent)
    }

    protected fun motor(name: String, config: DcMotorEx.() -> Unit = {}) = KMotor(
        hwMap[DcMotorEx::class.java, name].apply(config)
    ).registerDevice()

    protected fun servo(name: String, config: Servo.() -> Unit = {}) = KServo(
        hwMap[Servo::class.java, name].apply(config)
    ).registerDevice()

    protected fun crServo(name: String, config: CRServo.() -> Unit = {}) = KCRServo(
        hwMap[CRServo::class.java, name].apply(config)
    ).registerDevice()

    private var started = false
    private val timer = ElapsedTime()
    private val mainLog by lazy { Globals.log.hardware.sublog(hubName) }
    private val currentLog by lazy { mainLog.sublog("current") }

    fun syncHardware() {
        hub.clearBulkCache()
        callbacks.forEach { it.invoke() }

        mainLog["loop time (hz)"] = (1 / timer.seconds()).roundToInt()

        timer.reset()

        mainLog.collect()
    }

    fun readCurrents() {
        currentReadCallbacks.forEach { it.invoke() }

        if (started) currentLog["hub current (A)"] = hub.getCurrent(CurrentUnit.AMPS)
    }
}


