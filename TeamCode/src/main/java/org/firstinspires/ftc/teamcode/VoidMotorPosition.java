package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.lib.Hardware;

@TeleOp(name="Void Motor Position", group="9884")
public class VoidMotorPosition extends LinearOpMode {
    // Initialize hardware
    Hardware robot = new Hardware();
    // in ticks
    int clawPos = 0;

    @Override
    public void runOpMode() {
        // Initializes the hardware map
        robot.init(hardwareMap, telemetry);
        telemetry.addData("lmao", "lmao");
        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {
            robot.setArmPos(clawPos);

            if(gamepad1.dpad_up) {
                clawPos+=30;
            }
            else if(gamepad1.dpad_down) {
                clawPos-=15;
            }

            if(gamepad1.a) {
                telemetry.addData("pos: ", robot.ArmMotor.getCurrentPosition());
            }
            /*
            else if(gamepad1.left_bumper) {
                robot.setClawPos(clawPos-0.02);
                clawPos-=0.02;
            }
            else if(gamepad1.right_bumper) {
                robot.setClawPos(clawPos+0.02);
                clawPos+=0.02;
            }
            else if(gamepad1.b) {
                telemetry.addData("pos: ", robot.ClawLeft.getPosition());
            }
            */
        }
    }
}
