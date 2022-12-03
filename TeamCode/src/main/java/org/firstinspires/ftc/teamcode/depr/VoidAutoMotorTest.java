package org.firstinspires.ftc.teamcode.depr;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.lib.Hardware;


@Autonomous(name="Void Motor Test", group="9884")

public class VoidAutoMotorTest extends LinearOpMode {
    Hardware robot = new Hardware();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap, telemetry);
        telemetry.addData("lmao", "lmao");
        telemetry.update();

        waitForStart();
        robot.powerTime(0.2, 0, 0, 0, 2000);
        robot.powerTime(0, 0.2, 0, 0, 2000);
        robot.powerTime(0, 0, 0.2, 0, 2000);
        robot.powerTime(0, 0, 0, 0.2, 2000);

        robot.powerTime(0.2, 0.2, 0.2, 0.2, 5000);
    }
}
