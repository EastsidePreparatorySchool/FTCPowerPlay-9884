package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class Hardware {
    public DcMotor DriveMotorLF = null;
    public DcMotor DriveMotorRF = null;
    public DcMotor DriveMotorLB = null;
    public DcMotor DriveMotorRB = null;

    public void init(HardwareMap hwMap, Telemetry tele) {
        DriveMotorLF = hwMap.dcMotor.get("LF");
        DriveMotorRF = hwMap.dcMotor.get("RF");
        DriveMotorLB = hwMap.dcMotor.get("LB");
        DriveMotorRB = hwMap.dcMotor.get("RB");

        DriveMotorLF.setDirection(DcMotor.Direction.FORWARD);
        DriveMotorRF.setDirection(DcMotor.Direction.REVERSE);
        DriveMotorLB.setDirection(DcMotor.Direction.FORWARD);
        DriveMotorRB.setDirection(DcMotor.Direction.REVERSE);
    }
}
