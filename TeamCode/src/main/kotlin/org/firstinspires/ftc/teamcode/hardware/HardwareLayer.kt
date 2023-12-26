package org.firstinspires.ftc.teamcode.hardware

import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import org.firstinspires.ftc.teamcode.hardware.devices.KCRServo
import org.firstinspires.ftc.teamcode.hardware.devices.KDevice
import org.firstinspires.ftc.teamcode.hardware.devices.KMotor
import org.firstinspires.ftc.teamcode.hardware.devices.KServo
import org.firstinspires.ftc.teamcode.hardware.devices.KWebcam
import org.firstinspires.ftc.teamcode.util.ReadOnlyProperty
import org.firstinspires.ftc.teamcode.util.ReadWriteProperty
import org.openftc.easyopencv.OpenCvCameraFactory
import kotlin.math.roundToInt

/**
 * Centralized class for all hardware interactions.
 *
 * @param hwMap The [HardwareMap] to use.
 * @param hubName The name of the hub in the [HardwareMap].
 */
abstract class HardwareLayer(protected val hwMap: HardwareMap, hubName: String) {
    @Volatile var lastLoopFrequency = 0
    @Volatile var hubCurrent = 0.0

    /**
     * A list of callbacks to run when [syncHardware] is called.
     */
    private val callbacks = mutableListOf<() -> Unit>()

    /**
     * A list of callbacks to run when [readCurrents] is called.
     */
    private val currentReadCallbacks = mutableListOf<() -> Unit>()

    private val hub = hwMap[LynxModule::class.java, hubName]

    init {
        hub.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL
    }

    /**
     * Property delegate for a read-only field whose backing value is updated
     * when [syncHardware] is called.
     *
     * @param default The default value of the field.
     * @param getInput A function that returns the updated value for the field.
     */
    protected fun <T> inputField(default: T, getInput: () -> T) = object : ReadOnlyProperty<T> {
        override var value = default

        init {
            callbacks.add { value = getInput() }
        }
    }

    /**
     * Property delegate for a read-write field whose value is pushed when [syncHardware] is called.
     *
     * @param default The default value of the field.
     * @param setOutput Called when [syncHardware] is called to push the value of the field.
     */
    protected fun <T> outputField(default: T, setOutput: (T) -> Unit) = object :
        ReadWriteProperty<T> {
        override var value = default

        init {
            callbacks.add { setOutput(value) }
        }
    }

    /**
     * Registers a [KDevice] to be updated when [syncHardware] and [readCurrents] are called.
     */
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

    protected fun webcam(name: String, enablePreview: Boolean): KWebcam {
        val webcamName = hwMap[WebcamName::class.java, name]

        val camera = if (enablePreview) {
            val monitorId = hwMap.appContext.resources.getIdentifier(
                "cameraMonitorViewId", "id", hwMap.appContext.packageName
            )

            OpenCvCameraFactory.getInstance().createWebcam(webcamName, monitorId)
        } else OpenCvCameraFactory.getInstance().createWebcam(webcamName)

        return KWebcam(camera).registerDevice()
    }

    private val timer = ElapsedTime()

    /**
     * Issues hardware write and read commands.
     */
    fun syncHardware() {
        hub.clearBulkCache()
        callbacks.forEach { it.invoke() }

        lastLoopFrequency = (1.0 / timer.seconds()).roundToInt()
        timer.reset()
    }

    /**
     * Reads the current draw of all devices.
     */
    fun readCurrents() {
        hubCurrent = hub.getCurrent(CurrentUnit.AMPS)
        currentReadCallbacks.forEach { it.invoke() }
    }
}


