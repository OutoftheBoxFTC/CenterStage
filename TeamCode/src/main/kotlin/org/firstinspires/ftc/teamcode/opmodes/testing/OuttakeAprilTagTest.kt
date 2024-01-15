package org.firstinspires.ftc.teamcode.opmodes.testing

import arrow.fx.coroutines.raceN
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.outoftheboxrobotics.suspendftc.loopYieldWhile
import com.outoftheboxrobotics.suspendftc.suspendFor
import com.outoftheboxrobotics.suspendftc.suspendUntil
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import kotlinx.coroutines.Deferred
import kotlinx.coroutines.async
import kotlinx.coroutines.cancelAndJoin
import kotlinx.coroutines.coroutineScope
import kotlinx.coroutines.launch
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference
import org.firstinspires.ftc.robotcore.external.navigation.Orientation
import org.firstinspires.ftc.teamcode.RobotState
import org.firstinspires.ftc.teamcode.actions.hardware.ArmPosition
import org.firstinspires.ftc.teamcode.actions.hardware.DriveControlState
import org.firstinspires.ftc.teamcode.actions.hardware.closeClaws
import org.firstinspires.ftc.teamcode.actions.hardware.currentDrivePose
import org.firstinspires.ftc.teamcode.actions.hardware.driveControlState
import org.firstinspires.ftc.teamcode.actions.hardware.launchFixpoint
import org.firstinspires.ftc.teamcode.actions.hardware.launchOuttakeFixpoint
import org.firstinspires.ftc.teamcode.actions.hardware.nextBackboardApriltagPosition
import org.firstinspires.ftc.teamcode.actions.hardware.openClaws
import org.firstinspires.ftc.teamcode.actions.hardware.setArmPosition
import org.firstinspires.ftc.teamcode.driveState
import org.firstinspires.ftc.teamcode.opmodes.RobotOpMode
import org.firstinspires.ftc.teamcode.util.G
import org.firstinspires.ftc.teamcode.util.mainLoop
import org.firstinspires.ftc.teamcode.util.merge
import org.firstinspires.ftc.teamcode.util.set
import org.firstinspires.ftc.teamcode.vision.AprilTagDetectionPipeline
import org.firstinspires.ftc.teamcode.vision.PreloadDetectionPipeline
import org.firstinspires.ftc.teamcode.vision.aprilTagDetections
import org.firstinspires.ftc.teamcode.visionState
import org.openftc.apriltag.AprilTagDetection
import org.openftc.easyopencv.OpenCvCameraRotation
import kotlin.coroutines.coroutineContext

@Autonomous
class OuttakeAprilTagTest : RobotOpMode() {
    override suspend fun runSuspendOpMode() {
         closeClaws()
         setArmPosition(ArmPosition.OUTTAKE)

        G.chub.outtakeCamera.let {
            it.startCamera(640, 480, OpenCvCameraRotation.UPRIGHT)
            streamCamera(it)
            it.setPipeline(AprilTagDetectionPipeline.outtakePipeline())
        }

        var backboardPose: Pose2d? = Pose2d()
        val targetPos: Deferred<PreloadDetectionPipeline.RandomizationPosition>

        coroutineScope {
            targetPos = async {
                raceN(
                    coroutineContext,
                    {
                        suspendUntil { gamepad1.x }
                        PreloadDetectionPipeline.RandomizationPosition.LEFT
                    },
                    {
                        suspendUntil { gamepad1.y }
                        PreloadDetectionPipeline.RandomizationPosition.CENTER
                    },
                    {
                        suspendUntil { gamepad1.b }
                        PreloadDetectionPipeline.RandomizationPosition.RIGHT
                    }
                ).merge()
            }

            launch {
                while (!targetPos.isCompleted) {
                    backboardPose = nextBackboardApriltagPosition()
                }
            }

            loopYieldWhile({ !targetPos.isCompleted }) {
                val detections = G[RobotState.visionState.aprilTagDetections]

                detections.forEach {
                    telemetry.addLine(it.formatted())
                }

                backboardPose?.let {
                    telemetry["outtake"] = it
                    telemetry["delta"] = it - currentDrivePose()
                }
            }
        }

        val preOuttake = currentDrivePose()

        val (job, target) = launchOuttakeFixpoint(Pose2d(), targetPos.await())

        val returnPos = Pose2d(
            target.vec() + target.headingVec() * preOuttake.vec().distTo(target.vec()),
            preOuttake.heading
        )

        raceN(
            coroutineContext,
            {
                suspendFor(1000)
            },
            {
                suspendUntil { currentDrivePose().vec().distTo(target.vec()) < 1.0 }
                suspendFor(200)
            }
        )

        openClaws()
        job.cancelAndJoin()
        launchFixpoint(returnPos)

        mainLoop {  }
    }

    private fun AprilTagDetection.formatted(): String {
        val rot = Orientation.getOrientation(pose.R, AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES)

        return "AprilTagDetection(id=$id, x=${pose.x}, y=${pose.y}, z=${pose.z}, yaw=${rot.firstAngle}, pitch=${rot.secondAngle}"
    }
}