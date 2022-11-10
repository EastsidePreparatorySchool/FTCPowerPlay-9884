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

@Autonomous(name="Void Auto Cycle Blue", group="9884")
public class Autonomous2 extends LinearOpMode {
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

        //robot can start with one cone in hand? If so add in later.

        // potential sensor
        robot.powerTime(0.65, 0.65, 0.65, 0.65, 3000 );
        robot.turn(0.5, 1000);
        // loop cycle here
        robot.powerTime(0.65, 0.65, 0.65, 0.65, 2000);
        //arm raise slightly, open claw, close claw
        robot.powerTime(-0.65, -0.65, -0.65, -0.65, 3000);
        robot.turn(0.5, 1000);
        // arm raise, claw open
        // cycle

        robot.powerTime(0.65, 0.65, 0.65, 0.65, 1500);
        // probably raise arm for cone stack, then grab
        robot.powerTime(-0.65,-0.65, -0.65, -0.65, 2000); // to the junction
        robot.turn(0.5, 1500);
        // arm raise, lower.
        //times are estimated.
        //include strafe for navigating through cones and junctions.
    }
}
