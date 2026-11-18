package org.firstinspires.ftc.teamcode.teleop;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;
@Disabled
@TeleOp(name = "VisionTest", group = "Concept")
public class VisionTest extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException{

        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();

        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
//                .setCamera(BuiltinCameraDirection.BACK)
                .setCameraResolution(new Size(640, 480))
                .build();

        telemetry.addLine("Initialized. Waiting for start...");
        telemetry.update();

        // 2. Wait for start
        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {
            if (!tagProcessor.getDetections().isEmpty()) {
                AprilTagDetection tag = tagProcessor.getDetections().get(0);

                if (tag instanceof AprilTagSingleDetection && tag.ftcPose != null) {
                    AprilTagSingleDetection singleTag = (AprilTagSingleDetection) tag;
                    telemetry.addData("x", singleTag.ftcPose.x);
                    telemetry.addData("y", singleTag.ftcPose.y);
                    telemetry.addData("z", singleTag.ftcPose.z);
                    telemetry.addData("roll", singleTag.ftcPose.roll);
                    telemetry.addData("pitch", singleTag.ftcPose.pitch);
                    telemetry.addData("yaw", singleTag.ftcPose.yaw);
                    telemetry.addData("Id", singleTag.id);
                } else {
                    telemetry.addLine("Tag detected, but pose not available");
                }
            } else {
                telemetry.addLine("No AprilTags detected");
            }
            telemetry.update();
        }
    }
}
