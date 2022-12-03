package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.lib.Hardware;


@Autonomous(name="Void Time Auto Test", group="9884")

public class VoidTimeAutoTest extends LinearOpMode {
    Hardware robot = new Hardware();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap, telemetry, true);
        telemetry.addData("lmao", "lmao");
        telemetry.update();

        waitForStart();
        robot.strafeTimeInches(12, false);
    }
}
