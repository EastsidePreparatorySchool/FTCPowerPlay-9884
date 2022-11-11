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
    private boolean armBtnReleased = true;
    private boolean clawBtnReleased = true;

    @Override
    public void runOpMode() {
        // Initializes the hardware map
        robot.init(hardwareMap, telemetry);
        telemetry.addData("please tell me this works", "this works");
        telemetry.update();
        ElapsedTime runtime = new ElapsedTime();
        waitForStart();
        runtime.reset();
        /*
         * LS - translation, any direction IMP
         * RS - rotation IMP
         * LB - arm down (by cone height diff?) IMP
         * RB - arm up (by cone height diff?) IMP
         * LT - slow mode hold IMP
         * RT - grab, release toggle IMP
         * DPAD - forward, back, left, right IMP
         * A,B,X,Y - different set arm heights for junctions IMP
         * * A - default pickup height (0 in encoder) IMP
         * * B, X, Y - low, medium, high junctions respectively IMP
         */
        while (opModeIsActive()) {
            double lx = gamepad1.left_stick_x;
            double ly = -gamepad1.left_stick_y;
            double rx = gamepad1.right_stick_x;
            mult = 1;

            // slowmode

            if(gamepad1.left_trigger > 0.5) {
                mult = robot.SLOWMODE_CONSTANT;
            }

            // directional movement

            if(gamepad1.dpad_down || gamepad1.dpad_left || gamepad1.dpad_up || gamepad1.dpad_right) {
                power = robot.SPEED_CONSTANT*mult;
                if(gamepad1.dpad_up) {
                    robot.powerMotors(power, power, power, power);
                }
                if(gamepad1.dpad_down) {
                    robot.powerMotors(-power, -power, -power, -power);
                }
                if(gamepad1.dpad_right) {
                    robot.powerMotors(power, -power, -power, power);
                }
                if(gamepad1.dpad_left) {
                    robot.powerMotors(-power, power, power, -power);
                }
                continue;
            }

            // arm set pos code

            if(gamepad1.a) {
                robot.ArmMotor.setTargetPosition(0);
            }
            else if(gamepad1.b) {
                robot.ArmMotor.setTargetPosition(robot.LOW_JUNCTION_ENCODER_CONSTANT);
            }
            else if(gamepad1.x) {
                robot.ArmMotor.setTargetPosition(robot.MED_JUNCTION_ENCODER_CONSTANT);
            }
            else if(gamepad1.y) {
                robot.ArmMotor.setTargetPosition(robot.HIGH_JUNCTION_ENCODER_CONSTANT);
            }

            // arm increment code

            if(gamepad1.right_bumper && armBtnReleased) {
                armPos+= robot.ARM_INCREMENT_ENCODER_CONSTANT;
                if(armPos > robot.ARM_NEVER_EXCEED) {
                    armPos = robot.ARM_NEVER_EXCEED;
                }
                robot.ArmMotor.setTargetPosition(armPos);
                armBtnReleased=false;
            }
            else if(gamepad1.left_bumper && armBtnReleased) {
                armPos-= robot.ARM_INCREMENT_ENCODER_CONSTANT/2;
                if(armPos<0) {
                    armPos=0;
                }
                robot.ArmMotor.setTargetPosition(armPos);
                armBtnReleased=false;
            }

            if(!gamepad1.left_bumper && !gamepad1.right_bumper) {
                armBtnReleased = true;
            }

            // claw toggle code

            if(gamepad1.right_trigger > 0.5 && clawBtnReleased) {
                if(!clawClose) {
                    robot.setClawRot(robot.CLAW_CLOSED_POSITION);
                } else {
                    robot.setClawRot(robot.CLAW_OPEN_POSITION);
                }
                clawClose = !clawClose;
                clawBtnReleased = false;
            }

            if(gamepad1.right_trigger < 0.2) {
                clawBtnReleased = true;
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
            telemetry.addData("Arm Current Position", robot.ArmMotor.getCurrentPosition());
            telemetry.addData("Arm Target", armPos);
            telemetry.addData("Arm Internal Target", robot.ArmMotor.getTargetPosition());
            telemetry.update();
        }
    }
}
