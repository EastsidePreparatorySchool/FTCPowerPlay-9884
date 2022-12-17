package org.firstinspires.ftc.teamcode.lib;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

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

    public BNO055IMU imu = null;
    BNO055IMU.Parameters parameters = null;
    // Speed, arm height, claw position constan

    public final double SPEED_CONSTANT = 0.75;
    public final double SLOWMODE_CONSTANT = 0.33;
    public final double ARM_MOTOR_PPR = 1425.1;
    public final int LOW_JUNCTION_ENCODER_CONSTANT = (int)Math.round(ARM_MOTOR_PPR*4800/1425.1);
    public final int MED_JUNCTION_ENCODER_CONSTANT = (int)Math.round(ARM_MOTOR_PPR*7800/1425.1);
    public final int HIGH_JUNCTION_ENCODER_CONSTANT = (int)Math.round(ARM_MOTOR_PPR*11000/1425.1);
    public final int ARM_INCREMENT_ENCODER_CONSTANT = 600;
    public final int ARM_NEVER_EXCEED = HIGH_JUNCTION_ENCODER_CONSTANT;
    public final double LEFT_CLAW_OPEN_POSITION = 0.15;
    public final double RIGHT_CLAW_OPEN_POSITION = 0.2;
    public final double CLAW_CLOSED_POSITION = 0.34;

    // Values for encoder based auto

    public final int WHEEL_DIAMETER = 100;
    public final double WHEEL_PPR = 537.7;
    public final double WHEEL_CIRCUM_INCHES = (WHEEL_DIAMETER/25.4)*(Math.PI);
    public final double WHEEL_TICKS_PER_INCH = WHEEL_PPR/WHEEL_CIRCUM_INCHES;
    public final double WHEEL_FORWARD_MULTIPLIER = 0.96;
    public final double WHEEL_LATERAL_MULTIPLIER = 1.1;

    // CONSTANTS FOR TIME BASED AUTO :(
    public final double MS_PER_FOOT = 1100;
    public final double MS_PER_INCH = MS_PER_FOOT/12;
    public final double MS_LATERAL_MULTIPLIER = 1.76;

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

        imu = hwMap.get(BNO055IMU.class, "imu");
        parameters = new BNO055IMU.Parameters();
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
        for(DcMotor m : driveMotors) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        if(auto) {
            for(DcMotor m : driveMotors) {
                m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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

    public void driveTimeInches(double inches, boolean forwards) {
        if (forwards) {
            powerTime(SPEED_CONSTANT*SLOWMODE_CONSTANT,SPEED_CONSTANT*SLOWMODE_CONSTANT,SPEED_CONSTANT*SLOWMODE_CONSTANT,SPEED_CONSTANT*SLOWMODE_CONSTANT,(int)Math.round(inches*MS_PER_INCH));

        } else {
            powerTime(-SPEED_CONSTANT * SLOWMODE_CONSTANT, -SPEED_CONSTANT * SLOWMODE_CONSTANT, -SPEED_CONSTANT * SLOWMODE_CONSTANT, -SPEED_CONSTANT * SLOWMODE_CONSTANT, (int) Math.round(inches * MS_PER_INCH));
        }
    }

    public void strafeTimeInches(double inches, boolean right) {
        if(right){
            powerTime(SPEED_CONSTANT*SLOWMODE_CONSTANT,-SPEED_CONSTANT*SLOWMODE_CONSTANT,-SPEED_CONSTANT*SLOWMODE_CONSTANT,SPEED_CONSTANT*SLOWMODE_CONSTANT,(int)Math.round(inches*MS_PER_INCH*MS_LATERAL_MULTIPLIER));

        } else {
            powerTime(-SPEED_CONSTANT*SLOWMODE_CONSTANT,SPEED_CONSTANT*SLOWMODE_CONSTANT,SPEED_CONSTANT*SLOWMODE_CONSTANT,-SPEED_CONSTANT*SLOWMODE_CONSTANT,(int)Math.round(inches*MS_PER_INCH*MS_LATERAL_MULTIPLIER));

        }
    }

    // USE THESE FOR AUTO, TIME BASED AUTO IS NOT A GOOD IDEA
    // mvm we have to use time based auto for now, encoder cable broken

    public void turnDegrees(double power, double target, Telemetry tele) {
        float heading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        float absHeading=(heading+360)%360;
        float turn = (float)((target-absHeading+540)%360)-180;
        int dir = turn < 0 ? 1 : -1;
        // clock 1 counter -1
        power*=dir;
        for (DcMotor motor : driveMotors) {
            powerMotors(power, -power, power, -power);
        }
        while(!(absHeading>(target-1) && absHeading<(target+1))) {
            absHeading = (imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle+360)%360;
            tele.addData("internal heading",imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
            tele.addData("abs heading",absHeading);

        }
        for (DcMotor motor : driveMotors) {
            powerMotors(0, 0, 0, 0);
        }
    }

    public void driveBlindInches(double inches, double power, Telemetry tele) {
        double distDbl = inches * WHEEL_TICKS_PER_INCH * WHEEL_FORWARD_MULTIPLIER;
        double distRnd = Math.round(distDbl);
        int distInt = (int) distRnd;
        for(DcMotor motor : driveMotors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setPower(power);
        }
        while(Math.abs(DriveMotorFL.getCurrentPosition())<distInt) {
            tele.addData("pos", DriveMotorFR.getCurrentPosition());
            tele.addData("pos", DriveMotorBR.getCurrentPosition());
            tele.addData("pos", DriveMotorFL.getCurrentPosition());
            tele.addData("pos", DriveMotorBL.getCurrentPosition());
            tele.update();
        }
        for(DcMotor motor : driveMotors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setPower(0);
        }
        tele.addData("pos", DriveMotorFR.getCurrentPosition());
        tele.addData("pos", DriveMotorBR.getCurrentPosition());
        tele.addData("pos", DriveMotorFL.getCurrentPosition());
        tele.addData("pos", DriveMotorBL.getCurrentPosition());
        tele.update();
    }

    public void strafeBlindInches(double inches, double power, Telemetry tele) {
        double distDbl = inches * WHEEL_TICKS_PER_INCH * WHEEL_LATERAL_MULTIPLIER;
        double distRnd = Math.round(distDbl);
        int distInt = (int) distRnd;
        for(DcMotor motor : driveMotors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        DriveMotorFL.setPower(power);
        DriveMotorFR.setPower(-power);
        DriveMotorBL.setPower(-power);
        DriveMotorBR.setPower(power);

        while(Math.abs(DriveMotorFL.getCurrentPosition())<distInt) {
            tele.addData("pos", DriveMotorFR.getCurrentPosition());
            tele.addData("pos", DriveMotorBR.getCurrentPosition());
            tele.addData("pos", DriveMotorFL.getCurrentPosition());
            tele.addData("pos", DriveMotorBL.getCurrentPosition());
            tele.update();
        }
        for(DcMotor motor : driveMotors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setPower(0);
        }
        tele.addData("pos", DriveMotorFR.getCurrentPosition());
        tele.addData("pos", DriveMotorBR.getCurrentPosition());
        tele.addData("pos", DriveMotorFL.getCurrentPosition());
        tele.addData("pos", DriveMotorBL.getCurrentPosition());
        tele.update();
    }

    public void driveInches(double inches, Telemetry tele) {
        for(DcMotor motor : driveMotors) {
            double distDbl = inches * WHEEL_TICKS_PER_INCH * WHEEL_FORWARD_MULTIPLIER;
            double distRnd = Math.round(distDbl);
            int distInt = (int) distRnd;
            tele.addData("distance (double): ", distDbl);
            tele.addData("distance (rounded): ", distRnd);
            tele.addData("distance (int): ", distInt);
            motor.setTargetPosition(motor.getCurrentPosition() + distInt);
            tele.update();
        }

        while (DriveMotorFR.isBusy() || DriveMotorBR.isBusy() || DriveMotorFL.isBusy() || DriveMotorBL.isBusy()){
            tele.addData("pos", DriveMotorFR.getCurrentPosition());
            tele.addData("pos", DriveMotorBR.getCurrentPosition());
            tele.addData("pos", DriveMotorFL.getCurrentPosition());
            tele.addData("pos", DriveMotorBL.getCurrentPosition());
            tele.update();
        }
    }

    public void strafeInches(double inches, Telemetry tele) {
        double distDbl = inches * WHEEL_TICKS_PER_INCH * WHEEL_LATERAL_MULTIPLIER;
        double distRnd = Math.round(distDbl);
        int distInt = (int) distRnd;
        DriveMotorFL.setTargetPosition(DriveMotorFL.getCurrentPosition()+distInt);
        DriveMotorBL.setTargetPosition(DriveMotorBL.getCurrentPosition()-distInt);
        DriveMotorFR.setTargetPosition(DriveMotorFR.getCurrentPosition()-distInt);
        DriveMotorBR.setTargetPosition(DriveMotorBR.getCurrentPosition()+distInt);
        while (DriveMotorFR.isBusy() || DriveMotorBR.isBusy() || DriveMotorFL.isBusy() || DriveMotorBL.isBusy()){
            tele.addData("pos", DriveMotorFR.getCurrentPosition());
            tele.addData("pos", DriveMotorBR.getCurrentPosition());
            tele.addData("pos", DriveMotorFL.getCurrentPosition());
            tele.addData("pos", DriveMotorBL.getCurrentPosition());
            tele.update();
        }
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

    public void setClawRot(double left, double right) {
        ClawLeft.setPosition(left);
        ClawRight.setPosition(right);
    }
}
