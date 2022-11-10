package org.firstinspires.ftc.teamcode.lib;

import com.qualcomm.robotcore.hardware.DcMotor;
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

    public DcMotor ClawSlide = null;

    //public Servo ClawLeft = null;
    //public Servo ClawRight = null;
    public Servo claw = null;

    public final static double startClawPos = 0.0; // starting servo position

    public final double SPEED_CONSTANT = 0.65;
    public final double SLOWMODE_CONSTANT = 0.35;

    public Hardware() {}

    public void init(HardwareMap hwMap, Telemetry tele) {
        // Webcam = hwMap.get(WebcamName.class, "Webcam 1");

        DriveMotorFL = hwMap.dcMotor.get("FL");
        DriveMotorFR = hwMap.dcMotor.get("FR");
        DriveMotorBL = hwMap.dcMotor.get("BL");
        DriveMotorBR = hwMap.dcMotor.get("BR");

        // ArmMotor = hwMap.dcMotor.get("ARM");

        //ClawLeft = hwMap.servo.get("CLAWLEFT");
        //ClawRight = hwMap.servo.get("CLAWRIGHT");

        DriveMotorFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        DriveMotorFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        DriveMotorBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        DriveMotorBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // builders have CTE please colleges do not admit them for engineering
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
        powerTime(power, -power, -power, power, ms);
    }

    public void threadsleep(int ms) {try {Thread.sleep(ms);} catch (Exception e) {}}

    // ARM AND CLAW FUNCTIONS HERE

    public void setClawPos(double rot) {
    }
}
