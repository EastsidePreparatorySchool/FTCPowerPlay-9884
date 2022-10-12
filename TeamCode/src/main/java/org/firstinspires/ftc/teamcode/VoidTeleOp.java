package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Void TeleOp", group="9884")
public class VoidTeleOp extends LinearOpMode {
    // Initialize hardware
    Hardware robot = new Hardware();
    @Override
    public void runOpMode() {
        // Initializes the hardware map
        robot.init(hardwareMap, telemetry);
        ElapsedTime runtime = new ElapsedTime();
        waitForStart();
        runtime.reset();
        while (opModeIsActive()) {
            // Mecanum drive - left stick translation, right stick rotation (only x matters)
            double lx = gamepad1.left_stick_x;
            // y is inverted
            double ly = -gamepad1.left_stick_y;
            double rx = gamepad1.right_stick_x;
            // idk how this works tbh but the math checks out
            double powerFL=ly+lx+rx, powerFR=ly-lx-rx, powerBL=ly-lx+rx, powerBR=ly+lx-rx;
            // Normalize values within +-1 or will be truncated, keep ratios
            double normalize = Math.max(Math.abs(lx)+Math.abs(ly)+Math.abs(rx),1);
            powerFL/=normalize;
            powerFR/=normalize;
            powerBL/=normalize;
            powerBR/=normalize;
            // Power motors w/ values
            robot.powerMotors(powerFL,powerFR,powerBL,powerBR);
        }
    }
}
