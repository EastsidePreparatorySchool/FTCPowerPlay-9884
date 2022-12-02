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

@Autonomous(name="Void Auto Test", group="9884")
public class VoidAutoTest extends LinearOpMode {
    // INTRODUCE VARIABLES HERE
    Hardware robot = new Hardware();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap,telemetry,true);
        waitForStart();
        // PUT AUTON CODE HERE (DRIVER PRESSED THE PLAY BUTTON!)
        // APRILTAG VALUE IS "tagid"  - VALUE CORRESPONDS TO SIDE OF SIGNAL, IF -1 NO TAG FOUND AT ANY POINT
        robot.driveInches(15, telemetry);
    }
}
