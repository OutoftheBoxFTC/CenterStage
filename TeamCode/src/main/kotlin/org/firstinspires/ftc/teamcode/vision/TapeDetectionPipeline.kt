package org.firstinspires.ftc.teamcode.vision

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.geometry.times
import org.firstinspires.ftc.teamcode.RobotState
import org.firstinspires.ftc.teamcode.actions.hardware.extendoPose
import org.firstinspires.ftc.teamcode.driveState
import org.firstinspires.ftc.teamcode.util.G
import org.firstinspires.ftc.teamcode.util.deg
import org.firstinspires.ftc.teamcode.visionState
import org.opencv.core.Mat
import org.opencv.core.MatOfPoint
import org.opencv.core.Point
import org.opencv.core.Scalar
import org.opencv.core.Size
import org.opencv.imgproc.Imgproc
import org.openftc.easyopencv.OpenCvPipeline
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

        val h = 6.75
        val alpha = 15.0.deg

        val cameraOffset = Vector2d(-0.091, -4.005)

        @JvmField var threshold = 160.0

        @JvmField var minRectWidth = 20
        @JvmField var minRectHeight = 30
    }

    private val erodeKernel = Imgproc.getStructuringElement(
        Imgproc.MORPH_RECT,
        Size(9.0, 9.0)
    )
    private val blurSize = Size(5.0, 5.0)

    private val gray = Mat()
    private val thresh = Mat()
    private val blurred = Mat()
    private val contours = mutableListOf<MatOfPoint>()

    private val tapeQuad = Array(4) { Point() }

    var estimate: Vector2d? = null

    private var pointEstimate = Point(600.0, 600.0)

    var cameraPoseEstimate = Pose2d()

    override fun processFrame(input: Mat): Mat {
        var extendoPose: Pose2d

        // Threshold and find contours around tape
        Imgproc.cvtColor(input, gray, Imgproc.COLOR_RGB2GRAY)
        Imgproc.threshold(gray, thresh, threshold, 255.0, Imgproc.THRESH_BINARY)
        Imgproc.erode(thresh, thresh, erodeKernel)
        Imgproc.blur(thresh, blurred, blurSize)

        contours.clear()
        Imgproc.findContours(blurred, contours, Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE)

        Imgproc.drawContours(input, contours, -1, Scalar(255.0, 0.0, 0.0), 2)


        // Reproject estimate if available
        estimate?.let {
            extendoPose = G[RobotState.driveState.extendoPose]

            pointEstimate = reprojectPoint(
                (it - extendoPose.vec()).rotated(-extendoPose.heading) - cameraOffset
            )
        }

        // Find rect closest to estimate
        val rect = contours
            .map { Imgproc.boundingRect(it) }
            .filter { it.width >= minRectWidth && it.height >= minRectHeight }
            .onEach {
                Imgproc.rectangle(input, it, Scalar(255.0, 255.0, 0.0), 2)
            }
            .minByOrNull {
                hypot(pointEstimate.x - (it.x + it.width / 2), pointEstimate.y - (it.y + it.height))
            } ?: return input

        Imgproc.circle(input, pointEstimate, 10, Scalar(255.0, 0.0, 0.0), 4)
        Imgproc.rectangle(input, rect, Scalar(0.0, 255.0, 0.0), 2)

        // Get tape contour in submat
        val submat = blurred.submat(
            rect.y + 3*rect.height/4,
            rect.y + rect.height,
            rect.x,
            rect.x+rect.width
        )

        contours.clear()
        Imgproc.findContours(submat, contours, Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE)
        val bottomContourPoints = contours.first().toList()

        // Convert contour to quad
        val shiftPoint = Point(rect.x.toDouble(), rect.y + 3.0*rect.height/4)

        tapeQuad[0] = bottomContourPoints.minBy { it.x + it.y } + shiftPoint
        tapeQuad[1] = bottomContourPoints.maxBy { it.y - it.x } + shiftPoint
        tapeQuad[2] = bottomContourPoints.maxBy { it.y + it.x } + shiftPoint
        tapeQuad[3] = bottomContourPoints.minBy { it.y - it.x } + shiftPoint

        Imgproc.polylines(
            input, listOf(MatOfPoint().apply { fromArray(*tapeQuad) }),
            false, Scalar(0.0, 0.0, 255.0), 3
        )

        submat.release()

        // Calculate pose using lens intrinsics
        cameraPoseEstimate = poseFromQuad()

        extendoPose = G[RobotState.driveState.extendoPose]

        G[RobotState.visionState.stackTapePose] = extendoPose +
                Pose2d(
                    (cameraOffset + cameraPoseEstimate.vec()).rotated(extendoPose.heading),
                    cameraPoseEstimate.heading
                )

        return input
    }

    private fun mapPoint(pt: Point): Vector2d {
        val (u, v) = pt

        val theta = atan((v - cy) / fy)

        return Vector2d(
            h / tan(theta + alpha),
            -h * (u - cx) / (fx * sin(theta + alpha))
        )
    }

    private fun reprojectPoint(vec: Vector2d): Point {
        val (x, y) = vec

        val theta = atan(h / x) - alpha

        return Point(
            cx - y * (fx * sin(theta + alpha)) / h,
            tan(theta) * fy + cy
        )
    }

    private fun poseFromQuad(): Pose2d {
        val p0 = mapPoint(tapeQuad[0])
        val p1 = mapPoint(tapeQuad[1])
        val p2 = mapPoint(tapeQuad[2])
        val p3 = mapPoint(tapeQuad[3])

        val v = 0.5 * (p0 + p3) - 0.5 * (p1 + p2)

        return Pose2d(
            (p1 + p2) / 2.0,
            v.angle()
        )
    }

    private operator fun Point.plus(other: Point) = Point(x + other.x, y + other.y)

    private operator fun Point.component1() = x
    private operator fun Point.component2() = y
}