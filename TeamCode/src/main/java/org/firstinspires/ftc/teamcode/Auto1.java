package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import org.firstinspires.ftc.teamcode.lib.

@Autonomous(name="Auto1", group="Autonomous")
public class Auto1 extends LinearOpMode {
    Hardware robot = new Hardware();

    @Override
    public void runOpMode() {

        // Initializing
        robot.init(hardwareMap, telemetry);

        // telemetry

        telemetry.addData("Status", "Ready to run");
        telemetry.update();

        // wait for start
        waitForStart();
        // auto code
    }
}
