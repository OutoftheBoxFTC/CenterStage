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

/**
 * Pipeline for detecting location of the team prop for randomization.
 */
@Config
class PreloadDetectionPipeline : OpenCvPipeline() {
    enum class RandomizationPosition {
        LEFT,
        CENTER,
        RIGHT
    }

    companion object {
        // Center rect
        @JvmField var cx0 = 0.0
        @JvmField var cy0 = 0.0
        @JvmField var cx1 = 0.0
        @JvmField var cy1 = 0.0

        // Side rect
        @JvmField var sx0 = 0.0
        @JvmField var sy0 = 0.0
        @JvmField var sx1 = 0.0
        @JvmField var sy1 = 0.0

        // Thresholds
        @JvmField var redCutoff = 120
        @JvmField var blueCutoff = 120

        // Color to look for
        @JvmField var isBlue = true

        // Which side the side rect is on
        @JvmField var sideRectRight  = true
    }

    private val yCrCbMat = Mat()

    var mainBlueStrength = 0.0
    var sideBlueStrength = 0.0

    var mainRedStrength = 0.0
    var sideRedStrength = 0.0

    init {
        setBlueBack()
    }

    override fun processFrame(input: Mat): Mat {
        // Using YCbCr color space for thresholding with blue and red in specific regions
        // We can't actually see all three positions, so we default to the third position if the
        // other two are below the threshold.

        Imgproc.cvtColor(input, yCrCbMat, Imgproc.COLOR_BGR2YCrCb)

        val centerRect = Rect(Point(cx0, cy0), Point(cx1, cy1))
        val sideRect = Rect(Point(sx0, sy0), Point(sx1, sy1))

        Imgproc.rectangle(input, centerRect, Scalar(0.0, 255.0, 0.0))
        Imgproc.rectangle(input, sideRect, Scalar(255.0, 255.0, 0.0))

        val centerSubmat = yCrCbMat.submat(centerRect)
        val sideSubmat = yCrCbMat.submat(sideRect)

        // Could have used average here, but now I don't feel like changing it
        val centerSum = Core.sumElems(centerSubmat).`val`
        val sideSum = Core.sumElems(sideSubmat).`val`

        mainRedStrength = centerSum[1] / centerRect.area()
        mainBlueStrength = centerSum[2] / centerRect.area()

        sideRedStrength = sideSum[1] / sideRect.area()
        sideBlueStrength = sideSum[2] / sideRect.area()

        centerSubmat.release()
        sideSubmat.release()

        G[RobotState.visionState.preloadPosition] =
            when {
                sideRectRight && !isBlue -> when {
                    mainRedStrength < redCutoff -> RandomizationPosition.CENTER
                    sideRedStrength < redCutoff -> RandomizationPosition.RIGHT
                    else -> RandomizationPosition.LEFT
                }

                sideRectRight && isBlue -> when {
                    mainBlueStrength < blueCutoff -> RandomizationPosition.CENTER
                    sideBlueStrength < blueCutoff -> RandomizationPosition.RIGHT
                    else -> RandomizationPosition.LEFT
                }

                !sideRectRight && !isBlue -> when {
                    mainRedStrength < redCutoff -> RandomizationPosition.CENTER
                    sideRedStrength < redCutoff -> RandomizationPosition.LEFT
                    else -> RandomizationPosition.RIGHT
                }

                else -> when {
                    mainBlueStrength < blueCutoff -> RandomizationPosition.CENTER
                    sideBlueStrength < blueCutoff -> RandomizationPosition.LEFT
                    else -> RandomizationPosition.RIGHT
                }
            }

        return input
    }

    fun setRedBack() {
        cx0 = 320.0
        cx1 = 350.0
        cy0 = 380.0
        cy1 = 350.0

        sx0 = 610.0
        sx1 = 638.0
        sy0 = 350.0
        sy1 = 375.0

        isBlue = false
        sideRectRight = true
    }

    fun setBlueBack() {
        cx0 = 420.0
        cx1 = 450.0
        cy0 = 375.0
        cy1 = 345.0

        sx0 = 100.0
        sx1 = 125.0
        sy0 = 370.0
        sy1 = 395.0

        isBlue = true
        sideRectRight = false
    }

    fun setRedAud() {
        cx0 = 400.0
        cx1 = 430.0
        cy0 = 375.0
        cy1 = 345.0

        sx0 = 50.0
        sx1 = 80.0
        sy0 = 350.0
        sy1 = 380.0

        isBlue = false
        sideRectRight = false
    }

    fun setBlueAud() {
        cx0 = 275.0
        cx1 = 295.0
        cy0 = 360.0
        cy1 = 340.0

        sx0 = 610.0
        sx1 = 638.0
        sy0 = 350.0
        sy1 = 375.0

        isBlue = true
        sideRectRight = true
    }
}