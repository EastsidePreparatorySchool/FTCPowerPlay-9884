package org.firstinspires.ftc.teamcode.depr;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.lib.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.lib.Hardware;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
@Disabled
@Autonomous(name="Void Single Auto Right", group="9884")
public class VoidSingleAutoRight extends LinearOpMode {
    // INTRODUCE VARIABLES HERE
    Hardware robot = new Hardware();
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    int tagid = -1;

    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap,telemetry,true);
        robot.setClawRot(robot.CLAW_CLOSED_POSITION, robot.CLAW_CLOSED_POSITION);
        robot.ArmMotor.setTargetPosition(robot.LOW_JUNCTION_ENCODER_CONSTANT);
        // INIT CAMERA
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        // Initialize OpenCvCamera camera as webcam
        camera = OpenCvCameraFactory.getInstance().createWebcam(robot.Webcam, cameraMonitorViewId);
        // Initialize AprilTag pipeline
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(robot.tagsize, robot.fx, robot.fy, robot.cx, robot.cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(640,480, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            // elite error handling
            public void onError(int errorCode) {}
        });

        telemetry.setMsTransmissionInterval(50);

        telemetry.addData("lmao","lmao");
        telemetry.update();
        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         * BEFORE ROBOT STARTS
         */
        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();
            // SOME TAG FOUND
            if(currentDetections.size() != 0) {
                boolean tagFound = false;
                // CHECK IF TAGS ARE RELEVANT
                for(AprilTagDetection tag : currentDetections) {
                    // IF SO, TAG TO TELEM AND UPDATE tagid VARIABLE
                    if(tag.id == robot.LEFT || tag.id == robot.MIDDLE || tag.id == robot.RIGHT) {
                        tagOfInterest = tag;
                        tagid = tag.id;
                        tagFound = true;
                        break;
                    }
                }
                // TELEM IF RELEVANT TAG FOUND
                if(tagFound) {
                    telemetry.addData("Tag In Frame?", "Yes - " + tagid);
                    telemetry.addData("Last Tag Detected", tagid);
                }
            }
            // IF NO DETECTIONS, CHECK LAST RELEVANT TAG DETECTED
            else {
                telemetry.addData("Tag In Frame?", "No");
                if(tagid==-1) {
                    telemetry.addData("Last Tag Detected", "None");
                } else {
                    telemetry.addData("Last Tag Detected", tagid);
                }
            }
            telemetry.update();
            sleep(20);
        }
        // AFTER PLAY STARTED
        // CHECK IF TAG WAS EVER SIGHTED DURING INIT LOOP

        if(tagid!=-1) {
            telemetry.addData("Tag Sighted During Init", tagid);
        } else {
            telemetry.addLine("No Tag Sighted During Init");
        }

        // PUT AUTON CODE HERE (DRIVER PRESSED THE PLAY BUTTON!)
        // APRILTAG VALUE IS "tagid"  - VALUE CORRESPONDS TO SIDE OF SIGNAL, IF -1 NO TAG FOUND AT ANY POINT
        robot.driveInches(23.5, telemetry);
        robot.strafeInches(-35.25, telemetry);
        robot.ArmMotor.setTargetPosition(robot.HIGH_JUNCTION_ENCODER_CONSTANT);
        robot.driveInches(3, telemetry);
        robot.setClawRot(robot.LEFT_CLAW_OPEN_POSITION, robot.RIGHT_CLAW_OPEN_POSITION);
        robot.driveInches(-3, telemetry);
        robot.ArmMotor.setTargetPosition(0);
        switch (tagid) {
            case 1:
                robot.strafeInches(11.75, telemetry);
                break;
            case 2:
                robot.strafeInches(35.25, telemetry);
                break;
            case 3:
                robot.strafeInches(58.75, telemetry);
                break;
        }
    }
}
