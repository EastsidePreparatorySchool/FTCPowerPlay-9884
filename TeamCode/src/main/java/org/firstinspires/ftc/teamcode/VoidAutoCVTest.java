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
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.lib.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.lib.Hardware;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(name="Void CV Test", group="9884")
public class VoidAutoCVTest extends LinearOpMode {
    // INTRODUCE VARIABLES HERE
    Hardware robot = new Hardware();
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448
    // You will need to do your own calibration for other configurations!
    // ^^^^^ NOT MY COMMENTS
    // ima be real idk how to calibrate it so im just gonna hope these values work w the camera we have
    // we have logitech c270
    // figured it out (?), used values in res/xml/teamwebcalibration.xml

    double fx = 822.317;
    double fy = 822.317;
    double cx = 319.495;
    double cy = 242.502;

    // UNITS ARE METERS
    // assumed tag size - about 3.4 cm. not sure how this value plays into AprilTagDetectionPipeline but i sure hope it works
    // original value was .166
    double tagsize = 0.034;

    // Tag ID 1,2,3 from the 36h11 family

    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;
    int tagid = -1;

    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode() {
        // INIT CAMERA
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        // Initialize OpenCvCamera camera as webcam
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        // Initialize AprilTag pipeline
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

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

        robot.init(hardwareMap, telemetry);

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
                    if(tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT) {
                        tagOfInterest = tag;
                        tagid = tag.id;
                        tagFound = true;
                        break;
                    }
                }
                // TELEM IF RELEVANT TAG FOUND
                if(tagFound) {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                // IF NOT RELEVANT, CHECK LAST RELEVANT TAG DETECTED
                else {
                    telemetry.addLine("Don't see tag of interest :(");
                    if(tagOfInterest == null) {
                        telemetry.addLine("(A relevant tag has never been seen)");
                    }
                    else {
                        telemetry.addLine("\nBut we HAVE seen a relevant tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }
            }
            // IF NO DETECTIONS, CHECK LAST RELEVANT TAG DETECTED AND TELEM
            else {
                telemetry.addLine("Don't see tag of interest :(");
                if(tagOfInterest == null) {
                    telemetry.addLine("(A relevant tag has never been seen)");
                }
                else {
                    telemetry.addLine("\nBut we HAVE seen a relevant tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }
            }
            telemetry.update();
            sleep(20);
        }
        // AFTER PLAY STARTED
        // CHECK IF TAG WAS EVER SIGHTED DURING INIT LOOP
        // IF SO, TAG TO TELEM
        if(tagOfInterest != null) {
            telemetry.addLine("Most recent relevant tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        // IF NOT, be sad :(
        else {
            telemetry.addLine("No relevant tag snapshot available, one was never sighted during the init loop :(");
            telemetry.update();
        }

        // PUT AUTON CODE HERE (DRIVER PRESSED THE PLAY BUTTON!)
        // APRILTAG VALUE IS "tagid"  - VALUE CORRESPONDS TO SIDE OF SIGNAL, IF -1 NO TAG FOUND AT ANY POINT

    }
    // just sending tag data (rot, distance etc.) to telemetry
    void tagToTelemetry(AprilTagDetection detection) {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        /*
        Unnecessary info
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
        */
    }
}
