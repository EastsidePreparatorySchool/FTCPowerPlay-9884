package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.lib.Hardware;

@Autonomous(name="Void IMU Test", group="9884")
public class VoidImuTest extends LinearOpMode {
    Hardware robot = new Hardware();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, telemetry);

        telemetry.addData("Status", "Ready to Run ");
        telemetry.update();

        waitForStart();

        while(opModeIsActive()) {
            telemetry.addData("heading", robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
            telemetry.addData("absHeading",(robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle+360)%360);
            telemetry.update();
            robot.threadsleep(50);
        }
    }

}
