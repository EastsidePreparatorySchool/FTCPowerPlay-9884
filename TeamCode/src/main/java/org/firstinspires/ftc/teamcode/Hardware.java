package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class Hardware {
    public DcMotor DriveMotorFL = null;
    public DcMotor DriveMotorFR = null;
    public DcMotor DriveMotorBL = null;
    public DcMotor DriveMotorBR = null;

    public void init(HardwareMap hwMap, Telemetry tele) {
        DriveMotorFL = hwMap.dcMotor.get("FL");
        DriveMotorFR = hwMap.dcMotor.get("FR");
        DriveMotorBL = hwMap.dcMotor.get("BL");
        DriveMotorBR = hwMap.dcMotor.get("BR");

        DriveMotorFL.setDirection(DcMotor.Direction.FORWARD);
        DriveMotorFR.setDirection(DcMotor.Direction.REVERSE);
        DriveMotorBL.setDirection(DcMotor.Direction.FORWARD);
        DriveMotorBR.setDirection(DcMotor.Direction.REVERSE);
    }

    //basic functions

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

    public void turn(double power) {
        powerMotors(-power, power, -power, power);
    }

    public void turnTime(double power, int ms) {
        powerTime(-power, power, -power, power, ms);
    }

    public void strafe(double power) {
        powerMotors(power, -power, -power, power);
    }

    public void strafeTime(double power, int ms) {
        powerTime(power, -power, power, -power, ms);
    }

    public void sleep(int ms) {
        try {Thread.sleep(ms);} catch (Exception e) {}
    }
}
