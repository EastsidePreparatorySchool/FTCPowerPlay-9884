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
import org.firstinspires.ftc.teamcode.lib.Hardware;

@Autonomous(name="Void Auto Strafe Left Middle Parking", group="9884")

public class SimpleAutoStrafeLeftMidPark extends LinearOpMode {

    // declaring

    Hardware robot = new Hardware();

    @Override
    public void runOpMode() throws InterruptedException {

        // initializing

        robot.init(hardwareMap, telemetry);

        // telemetry

        telemetry.addData("Status", "Ready to run");
        telemetry.update();

        // wait for start

        waitForStart();

        // auto

        robot.strafe(-0.65, 3000); // tweak for full park
    }
}
