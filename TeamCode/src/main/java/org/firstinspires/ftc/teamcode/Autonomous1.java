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

@Autonomous(name="Autonomous1", group="Autonomous")
public class Autonomous1 extends LinearOpMode {
    Hardware robot = new Hardware();

    @Override
    public void runOpMode() throws InterruptedException {

        // Initializing

        robot.init(hardwareMap, telemetry);

        // telemetry

        telemetry.addData("Status", "Ready to run");
        telemetry.update();

        // wait for start
        waitForStart();

        // potential sensor
        robot.powerTime(0.65, 0.65, 0.65, 0.65, 4000 );
        robot.turn(0.5, 1000);
        robot.powerTime(0.65, 0.65, 0.65, 0.65, 1500);
        robot.powerTime(-0.65,-0.65, -0.65, -0.65, 2000);
        robot.turn(0.5, 1500);
        // arm raise, lower.
        //times are estimated.
        //include strafe for navigating through cones and junctions.
    }
}
