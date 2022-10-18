package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/*
* IMPORTANT
* ADD EOCV FILE TO ROBOT CONTROL HUB ASAP
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

        ClawSlide = hwMap.dcMotor.get("CLAWSLIDE");

        ClawLeft = hwMap.servo.get("CLAWLEFT");
        ClawRight = hwMap.servo.get("CLAWRIGHT");

        DriveMotorFL.setDirection(DcMotor.Direction.FORWARD);
        DriveMotorFR.setDirection(DcMotor.Direction.REVERSE);
        DriveMotorBL.setDirection(DcMotor.Direction.FORWARD);
        DriveMotorBR.setDirection(DcMotor.Direction.REVERSE);
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
        sleep(ms);
        powerMotors(0, 0, 0, 0);
    }

    // FOR TURN AND STRAFE, -power IS TURN/STRAFE LEFT, +power IS TURN/STRAFE RIGHT

    public void turn(double power, int ms) {
        powerTime(power, -power, power, -power, ms);
    }

    public void strafe(double power, int ms) {
        powerTime(power, -power, power, -power, ms);
    }

    public void sleep(int ms) {try {Thread.sleep(ms);} catch (Exception e) {}}

    // SLIDE AND CLAW FUNCTIONS HERE
}
