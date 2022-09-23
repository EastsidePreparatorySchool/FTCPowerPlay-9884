package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import java.util.*;

public class OpMode {
    @TeleOp(name="void robotics teleop", group="9884")
    public class TeleOpVoid extends LinearOpMode {
        //Initializes telemetry
        private ElapsedTime runtime = new ElapsedTime();

        @Override
        public void runOpMode() {
            //Initializes the hardware map
            Hardware robot = new Hardware();
            robot.init(hardwareMap, telemetry);

            //Wait for start (?)
            waitForStart();
            runtime.reset();
            //while (opModeIsActive()) {
                //controller

            //    if (gamepad1.dpad_right == true) strafe = -1;
            //    if (gamepad1.dpad_left == true) strafe = 1;
            //    if (gamepad1.dpad_up == true) drive = 1;
             //   if (gamepad1.dpad_down == true) drive = -1;
        }
    }
}
