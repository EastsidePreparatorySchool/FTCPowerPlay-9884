package org.firstinspires.ftc.teamcode.depr;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.lib.Hardware;

@Autonomous(name="Void Auto Cycle Red", group="9884")
public class VoidAutoRoute1 extends LinearOpMode {
    Hardware robot = new Hardware();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, telemetry);

        telemetry.addData("Status", "Ready to Run ");
        telemetry.update();

        waitForStart();

        robot.powerTime(0.65, 0.65, 0.65, 0.65, 4000);
        robot.turn(0.5, 1000);
        robot.powerTime(0.65, 0.65, 0.65, 0.65, 1500);
        robot.powerTime(-0.65, -0.65, -0.65, -0.65, 2000);
        robot.turn(0.5, 1500);

    }

}
