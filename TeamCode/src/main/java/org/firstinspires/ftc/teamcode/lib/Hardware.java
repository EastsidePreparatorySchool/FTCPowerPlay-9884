package org.firstinspires.ftc.teamcode.lib;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/*
* IMPORTANT
* ADD EOCV FILE TO CONTROL HUB FIRST FOLDER ASAP
* NEEDED FOR EOCV & APRILTAG DETECTION
*/

public class Hardware {
    public DcMotor DriveMotorFL = null;
    public DcMotor DriveMotorFR = null;
    public DcMotor DriveMotorBL = null;
    public DcMotor DriveMotorBR = null;

    public DcMotor ClawSlide = null;

    public Servo ClawLeft = null;
    public Servo ClawRight = null;

    public void init(HardwareMap hwMap, Telemetry tele) {
        DriveMotorFL = hwMap.dcMotor.get("FL");
        DriveMotorFR = hwMap.dcMotor.get("FR");
        DriveMotorBL = hwMap.dcMotor.get("BL");
        DriveMotorBR = hwMap.dcMotor.get("BR");

        //ClawSlide = hwMap.dcMotor.get("CLAWSLIDE");

        //ClawLeft = hwMap.servo.get("CLAWLEFT");
        //ClawRight = hwMap.servo.get("CLAWRIGHT");

        DriveMotorFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        DriveMotorFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        DriveMotorBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        DriveMotorBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // builders have CTE
        DriveMotorFL.setDirection(DcMotor.Direction.REVERSE);
        DriveMotorFR.setDirection(DcMotor.Direction.REVERSE);
        DriveMotorBL.setDirection(DcMotor.Direction.REVERSE);
        DriveMotorBR.setDirection(DcMotor.Direction.FORWARD);
    }

    // basic move functions for auto

    public void powerMotors(double powerFL, double powerFR, double powerBL, double powerBR) {
        DriveMotorFL.setPower(powerFL);
        DriveMotorFR.setPower(powerFR);
        DriveMotorBL.setPower(powerBL);
        DriveMotorBR.setPower(powerBR);
    }

    public void powerTime(double powerFL, double powerFR, double powerBL, double powerBR, int ms) {
        powerMotors(powerFL, powerFR, powerBL, powerBR);
        threadsleep(ms);
        powerMotors(0, 0, 0, 0);
    }

    // FOR TURN AND STRAFE, -power IS TURN/STRAFE LEFT, +power IS TURN/STRAFE RIGHT

    public void turn(double power, int ms) {
        powerTime(power, -power, power, -power, ms);
    }

    public void strafe(double power, int ms) {
        powerTime(power, -power, power, -power, ms);
    }

    public void threadsleep(int ms) {try {Thread.sleep(ms);} catch (Exception e) {}}

    // SLIDE AND CLAW FUNCTIONS HERE
}
