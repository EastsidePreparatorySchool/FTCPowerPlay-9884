package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.lib.Hardware;

@TeleOp(name="Void TeleOp", group="9884")
public class VoidTeleOp extends LinearOpMode {
    // Initialize hardware
    Hardware robot = new Hardware();

    private int armPos = 0;
    private double mult = 1;
    private double power = 0;
    private boolean clawClose = false;

    @Override
    public void runOpMode() {
        // Initializes the hardware map
        robot.init(hardwareMap, telemetry);
        telemetry.addData("lmao", "lmao");
        telemetry.update();
        ElapsedTime runtime = new ElapsedTime();
        waitForStart();
        runtime.reset();
        /*
         * LS - translation, any direction IMP
         * RS - rotation IMP
         * LB - arm down (by cone height diff?)
         * RB - arm up (by cone height diff?)
         * LT - slow mode hold IMP
         * RT - grab, release toggle
         * DPAD  - forward, back, left, right IMP
         * A,B,X,Y - different set arm heights for junctions
         * * A - default pickup height (0 in encoder)
         * * B, X, Y - low, medium, high junctions respectively
         */
        while (opModeIsActive()) {
            double lx = gamepad1.left_stick_x;
            double ly = -gamepad1.left_stick_y;
            double rx = gamepad1.right_stick_x;
            mult = 1;

            if(gamepad1.left_trigger > 0.5) {
                mult = robot.SLOWMODE_CONSTANT;
            }

            if(gamepad1.dpad_down || gamepad1.dpad_left || gamepad1.dpad_up || gamepad1.dpad_right) {
                power = robot.SPEED_CONSTANT*mult;
                if(gamepad1.dpad_up) {
                    robot.powerMotors(power, power, power, power);
                }
                if(gamepad1.dpad_down) {
                    robot.powerMotors(-power, -power, -power, -power);
                }
                if(gamepad1.dpad_right) {
                    robot.powerMotors(power, -power, power, -power);
                }
                if(gamepad1.dpad_left) {
                    robot.powerMotors(-power, power, -power, power);
                }
                continue;
            }

            // Adds vectors to get motor powers
            double powerFL=ly+lx+rx, powerFR=ly-lx-rx, powerBL=ly-lx+rx, powerBR=ly+lx-rx;
            // Normalize values within +- of default speed
            double normalize = 1;
            if(Math.abs(lx)+Math.abs(ly)+Math.abs(rx)>robot.SPEED_CONSTANT) {
                normalize = (Math.abs(lx)+Math.abs(ly)+Math.abs(rx))/robot.SPEED_CONSTANT;
            }

            normalize /= mult;
            powerFL /= normalize;
            powerFR /= normalize;
            powerBL /= normalize;
            powerBR /= normalize;

            robot.powerMotors(powerFL,powerFR,powerBL,powerBR);
        }
    }
}
