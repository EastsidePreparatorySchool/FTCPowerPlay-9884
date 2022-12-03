/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/*
 * im gonna try to make this code as readable as possible because its not very clear what it's doing at a first glance
 * i stole most of this code from the openftc eocv apriltags library
 * OpenFTC/EOCV-AprilTag-Plugin/examples
 * this uses the pipeline (unedited, in ./apriltagvision/AprilTagDetectionPipeline) and the AprilTagAutonomousInitDetectionExample.java file as a base
 */

/*
 * might refactor at some point to be more modular
 * package scanning for apriltag into .lib
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.lib.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.lib.Hardware;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(name="Void Auto Park", group="9884")
public class VoidAutoPark extends LinearOpMode {
    // INTRODUCE VARIABLES HERE
    Hardware robot = new Hardware();
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    int tagid = -1;

    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap,telemetry,true);
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
        robot.setClawRot(robot.LEFT_CLAW_OPEN_POSITION, robot.RIGHT_CLAW_OPEN_POSITION);
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
        robot.setClawRot(robot.CLAW_CLOSED_POSITION, robot.CLAW_CLOSED_POSITION);
        robot.driveTimeInches(27, true);
        switch (tagid) {
            case 1:
                robot.driveTimeInches(2.25, true);
                robot.strafeTimeInches(26, false);
                break;
            case 2:
                break;
            case 3:
                robot.driveTimeInches(2.25, false);
                robot.strafeTimeInches(28, true);
                break;
            case -1:
                break;

        }
    }
}