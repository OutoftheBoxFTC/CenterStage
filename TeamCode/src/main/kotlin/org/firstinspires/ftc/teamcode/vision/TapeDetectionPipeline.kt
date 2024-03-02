package org.firstinspires.ftc.teamcode.vision

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import org.firstinspires.ftc.teamcode.RobotState
import org.firstinspires.ftc.teamcode.util.G
import org.firstinspires.ftc.teamcode.visionState
import org.opencv.core.Mat
import org.opencv.core.MatOfPoint
import org.opencv.core.Point
import org.opencv.core.Scalar
import org.opencv.core.Size
import org.opencv.imgproc.Imgproc
import org.openftc.easyopencv.OpenCvPipeline
import kotlin.math.PI
import kotlin.math.atan
import kotlin.math.hypot
import kotlin.math.sin
import kotlin.math.tan

@Config
class TapeDetectionPipeline : OpenCvPipeline() {
    companion object {
        const val fx = 913.244775432
        const val fy = 913.244775432
        const val cx = 662.617336927
        const val cy = 467.953661671

        @JvmField var threshold = 160.0

        @JvmField var minRectWidth = 50
        @JvmField var minRectHeight = 50
    }

    private val erodeKernel = Imgproc.getStructuringElement(
        Imgproc.MORPH_RECT,
        Size(5.0, 5.0)
    )
    private val blurSize = Size(5.0, 5.0)

    private val gray = Mat()
    private val thresh = Mat()
    private val blurred = Mat()
    private val contours = mutableListOf<MatOfPoint>()

    private val tapeQuad = Array(4) { Point() }

    private var estimate = Point(800.0, 400.0)

    override fun processFrame(input: Mat): Mat {
        // Threshold and find contours around tape
        Imgproc.cvtColor(input, gray, Imgproc.COLOR_RGB2GRAY)
        Imgproc.threshold(gray, thresh, threshold, 255.0, Imgproc.THRESH_BINARY)
        Imgproc.erode(thresh, thresh, erodeKernel)
        Imgproc.blur(thresh, blurred, blurSize)

        Imgproc.findContours(blurred, contours, Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE)

        // Find rect closest to estimate
        val rect = contours
            .map { Imgproc.boundingRect(it) }
            .filter { it.width >= minRectWidth && it.height >= minRectHeight }
            .minByOrNull {
                hypot(estimate.x - it.x + it.width / 2, estimate.y - (it.y + it.width))
            } ?: return input

        Imgproc.circle(input, estimate, 10, Scalar(255.0, 0.0, 0.0), 4)
        Imgproc.rectangle(input, rect, Scalar(0.0, 255.0, 0.0), 2)

        // Get tape contour in submat
        val submat = blurred.submat(
            rect.y + 3*rect.height/4,
            rect.y + rect.height,
            rect.x,
            rect.x+rect.width
        )

        Imgproc.findContours(submat, contours, Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE)
        val bottomContourPoints = contours.first().toList()

        // Convert contour to quad
        val shiftPoint = Point(rect.x.toDouble(), rect.y + 3.0*rect.height/4)

        tapeQuad[0] = bottomContourPoints.minBy { it.x + it.y } + shiftPoint
        tapeQuad[1] = bottomContourPoints.maxBy { it.y - it.x } + shiftPoint
        tapeQuad[2] = bottomContourPoints.maxBy { it.y + it.y } + shiftPoint
        tapeQuad[3] = bottomContourPoints.minBy { it.y - it.x } + shiftPoint

        Imgproc.polylines(
            input, listOf(MatOfPoint().apply { fromArray(*tapeQuad) }),
            false, Scalar(0.0, 0.0, 255.0), 2
        )

        estimate.x = (tapeQuad[1].x + tapeQuad[2].x) / 2
        estimate.y = (tapeQuad[1].y + tapeQuad[2].y) / 2

        submat.release()

        // Calculate pose using lens intrinsics
        G[RobotState.visionState.stackTapePose] = poseFromQuad()

        return input
    }

    private fun mapPoint(pt: Point): Vector2d {
        val (u, v) = pt

        val theta = atan((v - cy) / fy)

        val h = 5.5
        val alpha = PI / 6

        return Vector2d(
            h / tan(theta + alpha)
            -h * (u - cx) / (fx * sin(theta + alpha))
        )
    }

    private fun poseFromQuad(): Pose2d {
        val p0 = mapPoint(tapeQuad[0])
        val p1 = mapPoint(tapeQuad[1])
        val p2 = mapPoint(tapeQuad[2])
        val p3 = mapPoint(tapeQuad[3])

        val v0 = p0 - p1
        val v2 = p3 - p2

        return Pose2d(
            (p1 + p2) / 2.0,
            (v0.angle() + v2.angle()) / 2.0
        )
    }

    private operator fun Point.plus(other: Point) = Point(x + other.x, y + other.y)

    private operator fun Point.component1() = x
    private operator fun Point.component2() = y
}