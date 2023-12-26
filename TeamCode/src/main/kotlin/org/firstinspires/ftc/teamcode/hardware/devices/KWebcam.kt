package org.firstinspires.ftc.teamcode.hardware.devices

import arrow.core.getOrElse
import arrow.core.left
import arrow.core.right
import kotlinx.coroutines.suspendCancellableCoroutine
import org.openftc.easyopencv.OpenCvCamera
import org.openftc.easyopencv.OpenCvCameraRotation
import org.openftc.easyopencv.OpenCvWebcam
import kotlin.coroutines.resume

/**
 * Wrapper for [OpenCvWebcam] that implements [KDevice].
 */
class KWebcam(private val camera: OpenCvWebcam) : KDevice, OpenCvWebcam by camera {
    override fun writeData() = Unit
    override fun readCurrent() = 0.0

    /**
     * OpenCameraDevice implemented as a suspend function.
     */
    suspend fun startCamera(
        width: Int,
        height: Int,
        rot: OpenCvCameraRotation = OpenCvCameraRotation.UPRIGHT
    ): Unit = suspendCancellableCoroutine {
        openCameraDeviceAsync(object : OpenCvCamera.AsyncCameraOpenListener {
            override fun onOpened() = it.resume(Unit.right())
            override fun onError(errorCode: Int) = it.resume(errorCode.left())
        })
    }
        .map { startStreaming(width, height, rot) }
        .getOrElse { error("OpenCV Camera Open Error: $it") }
}