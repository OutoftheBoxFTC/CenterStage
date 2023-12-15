package org.firstinspires.ftc.teamcode.vision

import com.acmerobotics.dashboard.config.Config
import org.firstinspires.ftc.teamcode.RobotState
import org.firstinspires.ftc.teamcode.util.G
import org.firstinspires.ftc.teamcode.visionState
import org.opencv.core.Core
import org.opencv.core.Mat
import org.opencv.core.Point
import org.opencv.core.Rect
import org.opencv.core.Scalar
import org.opencv.imgproc.Imgproc
import org.openftc.easyopencv.OpenCvPipeline

@Config
class PreloadDetectionPipeline : OpenCvPipeline() {
    enum class RandomizationPosition {
        LEFT,
        CENTER,
        RIGHT
    }

    companion object {
        @JvmField var cx0 = 0.0
        @JvmField var cy0 = 0.0
        @JvmField var cx1 = 0.0
        @JvmField var cy1 = 0.0

        @JvmField var sx0 = 0.0
        @JvmField var sy0 = 0.0
        @JvmField var sx1 = 0.0
        @JvmField var sy1 = 0.0

        @JvmField var redCutoff = 120
        @JvmField var blueCutoff = 120

        @JvmField var isBlue = true
    }

    private val yCrCbMat = Mat()

    var mainBlueStrength = 0.0
    var sideBlueStrength = 0.0

    var mainRedStrength = 0.0
    var sideRedStrength = 0.0

    init {
        setBlue()
    }

    override fun processFrame(input: Mat): Mat {
        Imgproc.cvtColor(input, yCrCbMat, Imgproc.COLOR_BGR2YCrCb)

        val centerRect = Rect(Point(cx0, cy0), Point(cx1, cy1))
        val sideRect = Rect(Point(sx0, sy0), Point(sx1, sy1))

        Imgproc.rectangle(input, centerRect, Scalar(0.0, 255.0, 0.0))
        Imgproc.rectangle(input, sideRect, Scalar(255.0, 255.0, 0.0))

        val centerSubmat = yCrCbMat.submat(centerRect)
        val sideSubmat = yCrCbMat.submat(sideRect)

        val centerSum = Core.sumElems(centerSubmat).`val`
        val sideSum = Core.sumElems(sideSubmat).`val`

        mainRedStrength = centerSum[1] / centerRect.area()
        mainBlueStrength = centerSum[2] / centerRect.area()

        sideRedStrength = sideSum[1] / sideRect.area()
        sideBlueStrength = sideSum[2] / sideRect.area()

        centerSubmat.release()
        sideSubmat.release()

        G[RobotState.visionState.preloadPosition] =
            if (isBlue) when {
                mainBlueStrength < blueCutoff -> RandomizationPosition.CENTER
                sideBlueStrength < blueCutoff -> RandomizationPosition.LEFT
                else -> RandomizationPosition.RIGHT
            }

            else when {
                mainRedStrength < redCutoff -> RandomizationPosition.CENTER
                sideRedStrength < redCutoff -> RandomizationPosition.RIGHT
                else -> RandomizationPosition.LEFT
            }

        return input
    }

    fun setRed() {
        cx0 = 320.0
        cx1 = 350.0
        cy0 = 380.0
        cy1 = 350.0

        sx0 = 610.0
        sx1 = 638.0
        sy0 = 350.0
        sy1 = 375.0

        isBlue = false
    }

    fun setBlue() {
        cx0 = 420.0
        cx1 = 450.0
        cy0 = 375.0
        cy1 = 345.0

        sx0 = 100.0
        sx1 = 125.0
        sy0 = 370.0
        sy1 = 395.0

        isBlue = true
    }
}