package org.firstinspires.ftc.teamcode.depr;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.lib.Hardware;
@Disabled
@Autonomous(name="Void Auto Strafe Right", group="9884")
public class VoidAutoStrafeRight extends LinearOpMode {
    Hardware robot = new Hardware();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, telemetry);
        waitForStart();
        robot.strafe(0.5,2000);
    }
}
