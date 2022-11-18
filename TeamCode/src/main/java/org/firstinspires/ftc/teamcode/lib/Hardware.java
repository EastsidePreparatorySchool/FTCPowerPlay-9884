package org.firstinspires.ftc.teamcode.lib;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

public class Hardware {

    public DcMotor[] driveMotors;

    public WebcamName Webcam = null;

    public DcMotor DriveMotorFL = null;
    public DcMotor DriveMotorFR = null;
    public DcMotor DriveMotorBL = null;
    public DcMotor DriveMotorBR = null;

    public DcMotor ArmMotor = null;

    public Servo ClawLeft = null;
    public Servo ClawRight = null;

    // Speed, arm height, claw position constants

    public final double SPEED_CONSTANT = 0.5;
    public final double SLOWMODE_CONSTANT = 0.5;
    public final int LOW_JUNCTION_ENCODER_CONSTANT = 4800;
    public final int MED_JUNCTION_ENCODER_CONSTANT = 7800;
    public final int HIGH_JUNCTION_ENCODER_CONSTANT = 11000;
    public final int ARM_INCREMENT_ENCODER_CONSTANT = 600;
    public final int ARM_NEVER_EXCEED = 11000;
    public final double CLAW_OPEN_POSITION = 0.14;
    public final double CLAW_CLOSED_POSITION = 0.34;

    // Values for encoder based auto

    public final int WHEEL_DIAMETER = 100;
    public final double WHEEL_PPR = 145.1;
    public final double WHEEL_CIRCUM_INCHES = (WHEEL_DIAMETER/25.4)*(Math.PI);
    public final double WHEEL_TICKS_PER_INCH = WHEEL_CIRCUM_INCHES/WHEEL_PPR;
    public final int WHEEL_LATERAL_MULTIPLIER = 1;

    // Logitech C270 lens intrinsics
    // /TeamCode/src/main/res/xml/teamwebcamcalibrations.xml

    public final double fx = 822.317;
    public final double fy = 822.317;
    public final double cx = 319.495;
    public final double cy = 242.502;

    // UNITS ARE METERS

    public final double tagsize = 0.051;

    // Tag ID 1,2,3 from the 36h11 family

    public final int LEFT = 1;
    public final int MIDDLE = 2;
    public final int RIGHT = 3;

    public Hardware() {}

    // overload init method, default no auto
    public void init(HardwareMap hwMap, Telemetry tele) {
        init(hwMap, tele, false);
    }

    public void init(HardwareMap hwMap, Telemetry tele, boolean auto) {
        Webcam = hwMap.get(WebcamName.class, "Webcam 1");

        BNO055IMU imu = hwMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(parameters);

        DriveMotorFL = hwMap.dcMotor.get("FL");
        DriveMotorFR = hwMap.dcMotor.get("FR");
        DriveMotorBL = hwMap.dcMotor.get("BL");
        DriveMotorBR = hwMap.dcMotor.get("BR");

        driveMotors = new DcMotor[]{DriveMotorFL,DriveMotorFR,DriveMotorBL,DriveMotorBR};

        ArmMotor = hwMap.dcMotor.get("ARM");

        ClawLeft = hwMap.servo.get("CLAWLEFT");
        ClawRight = hwMap.servo.get("CLAWRIGHT");

        ClawRight.setDirection(Servo.Direction.REVERSE);

        for(DcMotor motor : driveMotors) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

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

        if(auto) {
            for(DcMotor motor : driveMotors) {
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motor.setTargetPosition(0);
                motor.setPower(0.65);
            }
        }
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

    public void driveTime(double power, int ms) {
        powerMotors(power, power, power, power);
        threadsleep(ms);
        powerMotors(0, 0, 0, 0);
    }

    // USE THESE FOR AUTO, TIME BASED AUTO IS NOT A GOOD IDEA

    public void driveInches(double inches) {
        for(DcMotor motor : driveMotors) {
            motor.setTargetPosition(motor.getCurrentPosition()+(int)Math.round(inches*WHEEL_TICKS_PER_INCH));
        }
    }

    public void strafeInches(double inches) {
        DriveMotorFL.setTargetPosition(DriveMotorFL.getCurrentPosition()+(int)Math.round(inches*WHEEL_TICKS_PER_INCH*WHEEL_LATERAL_MULTIPLIER));
        DriveMotorBL.setTargetPosition(DriveMotorBL.getCurrentPosition()-(int)Math.round(inches*WHEEL_TICKS_PER_INCH*WHEEL_LATERAL_MULTIPLIER));
        DriveMotorFR.setTargetPosition(DriveMotorFR.getCurrentPosition()-(int)Math.round(inches*WHEEL_TICKS_PER_INCH*WHEEL_LATERAL_MULTIPLIER));
        DriveMotorBR.setTargetPosition(DriveMotorBR.getCurrentPosition()+(int)Math.round(inches*WHEEL_TICKS_PER_INCH*WHEEL_LATERAL_MULTIPLIER));
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
