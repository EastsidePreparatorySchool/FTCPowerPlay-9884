package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.lib.Hardware;

@TeleOp(name="Void Motor Position", group="9884")
public class VoidMotorPosition extends LinearOpMode {
    // Initialize hardware
    Hardware robot = new Hardware();
    // in ticks
    int armPos = 0;
    double clawPos = robot.CLAW_OPEN_POSITION;
    boolean armBtnReleased = true;
    boolean clawBtnReleased = true;

    @Override
    public void runOpMode() {
        // Initializes the hardware map
        robot.init(hardwareMap, telemetry);
        telemetry.update();
        waitForStart();
        robot.setClawRot(robot.CLAW_OPEN_POSITION);
        while (opModeIsActive()) {

            if(gamepad1.right_trigger > 0.5 && armBtnReleased) {
                armPos += robot.ARM_INCREMENT_ENCODER_CONSTANT;
                if(armPos > robot.ARM_NEVER_EXCEED) {
                    armPos = robot.ARM_NEVER_EXCEED;
                }
                robot.ArmMotor.setTargetPosition(armPos);
                armBtnReleased=false;
            }
            else if(gamepad1.left_trigger > 0.5 && armBtnReleased) {
                armPos -= robot.ARM_INCREMENT_ENCODER_CONSTANT/2;
                if(armPos < 0) {
                    armPos = 0;
                }
                robot.ArmMotor.setTargetPosition(armPos);
                armBtnReleased=false;
            }
            if(gamepad1.left_trigger < 0.5 && gamepad1.right_trigger < 0.5) {
                armBtnReleased = true;
            }



            if(gamepad1.left_bumper && clawBtnReleased) {
                if(clawPos-0.01<0) {
                    clawPos+=0.01;
                }
                robot.setClawRot(clawPos-0.01);
                clawPos-=0.01;
                clawBtnReleased=false;
            }
            else if(gamepad1.right_bumper && clawBtnReleased) {
                robot.setClawRot(clawPos+0.01);
                clawPos+=0.01;
                clawBtnReleased=false;
            }

            if(!gamepad1.left_bumper && !gamepad1.right_bumper) {
                clawBtnReleased = true;
            }

            if (gamepad1.a) {
                robot.setClawRot(robot.CLAW_OPEN_POSITION);
                clawPos= robot.CLAW_OPEN_POSITION;
            }
            if (gamepad1.b) {
                robot.setClawRot(robot.CLAW_CLOSED_POSITION);
                clawPos= robot.CLAW_CLOSED_POSITION;
            }



            telemetry.addData("Arm Current Position", robot.ArmMotor.getCurrentPosition());
            telemetry.addData("Arm Target", armPos);
            telemetry.addData("Arm Internal Target", robot.ArmMotor.getTargetPosition());

            telemetry.addData("Claw Pos", clawPos);

            telemetry.update();
        }
    }
}
