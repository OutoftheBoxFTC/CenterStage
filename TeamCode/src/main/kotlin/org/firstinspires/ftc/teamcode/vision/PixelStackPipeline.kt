package org.firstinspires.ftc.teamcode.vision

import com.acmerobotics.dashboard.config.Config
import org.firstinspires.ftc.teamcode.RobotState
import org.firstinspires.ftc.teamcode.util.G
import org.firstinspires.ftc.teamcode.visionState
import org.opencv.core.Mat
import org.opencv.core.MatOfPoint
import org.opencv.core.Rect
import org.opencv.core.Scalar
import org.opencv.core.Size
import org.opencv.imgproc.Imgproc
import org.openftc.easyopencv.OpenCvPipeline
import kotlin.math.abs

// Will be rewritten at some point. Detects where the pixel stack is.
@Config
@Suppress("PrivatePropertyName")
class PixelStackPipeline : OpenCvPipeline() {
    private val blurSize = Size(5.0, 5.0)

    private val WHITE = Scalar(255.0, 255.0, 255.0)
    private val RED = Scalar(255.0, 0.0, 0.0)

    private val contours = mutableListOf<MatOfPoint>()
    private val grayscale = Mat()
    private val threshold = Mat()
    private val blurred = Mat()
    override fun processFrame(input: Mat): Mat {
        Imgproc.cvtColor(input, grayscale, Imgproc.COLOR_RGB2GRAY)
        Imgproc.threshold(
            grayscale, threshold, thresholdValue.toDouble(), 255.0, Imgproc.THRESH_BINARY
        )
        Imgproc.blur(threshold, blurred, blurSize)
        contours.clear()
        Imgproc.findContours(
            blurred, contours, Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE
        )

        val rect = contours
            .asSequence()
            .map(Imgproc::boundingRect)
            .filter(::filterInitSize)
            .sortedBy { -it.y }
            .take(3)
            .toList()
            .onEach { Imgproc.rectangle(input, it, WHITE) }
            .minByOrNull { abs(input.width() / 2 - rectMid(it)) }

        if (rect != null) Imgproc.rectangle(input, rect, RED)

        G[RobotState.visionState.nullablePixelStackPosition] = rect?.let(::rectMid)

        return input
    }

    private fun filterInitSize(rect: Rect): Boolean {
        return rect.height >= minRectHeight && rect.width >= minRectWidth
    }

    private fun rectMid(rect: Rect): Int {
        return rect.x + rect.width / 2
    }

    companion object {
        @JvmField var thresholdValue = 200
        @JvmField var minRectHeight = 20
        @JvmField var minRectWidth = 20
    }
}