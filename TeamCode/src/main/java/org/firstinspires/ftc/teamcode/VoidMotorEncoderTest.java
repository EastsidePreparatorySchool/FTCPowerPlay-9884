package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.lib.Hardware;

import java.util.ArrayList;

@Autonomous(name="Void Motor Encoder Test", group="9884")
public class VoidMotorEncoderTest extends LinearOpMode {
    Hardware robot = new Hardware();
    @Override
    public void runOpMode() {
        robot.init(hardwareMap, telemetry, true);
        waitForStart();
        robot.driveInches(12, telemetry);
    }
}
