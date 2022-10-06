package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class OpMode {
    @TeleOp(name="Void Robotics Teleop", group="9884")
    public class TeleOpVoid extends LinearOpMode {
        Hardware robot = new Hardware();
        @Override
        public void runOpMode() {
            //Initializes the hardware map
            robot.init(hardwareMap, telemetry);
            ElapsedTime runtime = new ElapsedTime();
            //Wait for start (?)
            waitForStart();
            runtime.reset();
            while (opModeIsActive()) {
                //Mecanum drive - left stick translation, right stick rotation (only x value)
                double lx = gamepad1.left_stick_x;
                double ly = -gamepad1.left_stick_y;
                double rx = gamepad1.right_stick_x;
                //idk how this works google is a magical place
                double powerFL=ly+lx+rx, powerFR=ly-lx-rx, powerBL=ly-lx+rx, powerBR=ly+lx-rx;
                //Normalize values within -1,+1, keep ratios
                double normalize = Math.max(Math.abs(lx)+Math.abs(ly)+Math.abs(rx),1);
                powerFL/=normalize;
                powerFR/=normalize;
                powerBL/=normalize;
                powerBR/=normalize;
                //Power motors
                robot.powerMotors(powerFL,powerFR,powerBL,powerBR);
            }
        }
    }
}
