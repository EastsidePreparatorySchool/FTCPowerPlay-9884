package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.lib.Hardware;

import java.util.concurrent.TimeUnit;

@Autonomous(name="Void Servo Test", group="9884")

public class VoidClawServoTest extends LinearOpMode {
    Hardware robot = new Hardware();

    boolean clawBtnReleased = true;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, telemetry);
        telemetry.addData("Status", "Ready to run");
        telemetry.update();

        waitForStart();

        robot.ClawLeft.setPosition(0.4);
        robot.ClawRight.setPosition(0.4);

        robot.ClawLeft.setDirection(Servo.Direction.REVERSE);
        robot.ClawRight.setDirection(Servo.Direction.FORWARD);

        while(opModeIsActive()) {
            // if(gamepad1.right_trigger){

            //}
        }
    }
}
