package org.firstinspires.ftc.teamcode.lib;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

public class Hardware {
    public WebcamName Webcam = null;

    public DcMotor DriveMotorFL = null;
    public DcMotor DriveMotorFR = null;
    public DcMotor DriveMotorBL = null;
    public DcMotor DriveMotorBR = null;

    public DcMotor ArmMotor = null;

    public Servo ClawLeft = null;
    public Servo ClawRight = null;

    public final double SPEED_CONSTANT = 0.5;
    public final double SLOWMODE_CONSTANT = 0.5;
    public final int LOW_JUNCTION_ENCODER_CONSTANT = 4800;
    public final int MED_JUNCTION_ENCODER_CONSTANT = 7800;
    public final int HIGH_JUNCTION_ENCODER_CONSTANT = 11000;
    public final int ARM_INCREMENT_ENCODER_CONSTANT = 600;
    public final int ARM_NEVER_EXCEED = 11000;
    public final double CLAW_OPEN_POSITION = 0.14;
    public final double CLAW_CLOSED_POSITION = 0.33;


    public Hardware() {}

    public void init(HardwareMap hwMap, Telemetry tele) {
        // Webcam = hwMap.get(WebcamName.class, "Webcam 1");

        DriveMotorFL = hwMap.dcMotor.get("FL");
        DriveMotorFR = hwMap.dcMotor.get("FR");
        DriveMotorBL = hwMap.dcMotor.get("BL");
        DriveMotorBR = hwMap.dcMotor.get("BR");

        ArmMotor = hwMap.dcMotor.get("ARM");

        ClawLeft = hwMap.servo.get("CLAWLEFT");
        ClawRight = hwMap.servo.get("CLAWRIGHT");

        ClawRight.setDirection(Servo.Direction.REVERSE);

        DriveMotorFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        DriveMotorFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        DriveMotorBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        DriveMotorBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        ArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ArmMotor.setTargetPosition(0);
        ArmMotor.setPower(1);
        ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // builders have CTE please colleges do not admit them for engineering
        // im pretty sure the internal direction of some motors is reversed or something not sure
        DriveMotorFL.setDirection(DcMotor.Direction.REVERSE);
        DriveMotorFR.setDirection(DcMotor.Direction.REVERSE);
        DriveMotorBL.setDirection(DcMotor.Direction.REVERSE);
        DriveMotorBR.setDirection(DcMotor.Direction.FORWARD);

        ArmMotor.setDirection(DcMotorSimple.Direction.REVERSE);
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
        powerTime(power, -power, -power, power, ms);
    }

    public void threadsleep(int ms) {try {Thread.sleep(ms);} catch (Exception e) {}}

    // ARM AND CLAW FUNCTIONS HERE

    public void setClawRot(double rot) {
        ClawLeft.setPosition(rot);
        ClawRight.setPosition(rot);
    }
}
