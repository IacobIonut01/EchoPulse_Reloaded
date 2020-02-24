package org.firstinspires.ftc.teamcode.disabled;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.motors.RevRoboticsCoreHexMotor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

public class EchoPulse_Parts {

    private HardwareMap hardwareMap;

    EchoPulse_Parts(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    DcMotor getMotorSF() {
        return hardwareMap.dcMotor.get("MotorSF");
    }

    DcMotor getMotorDF() {
        return hardwareMap.dcMotor.get("MotorDF");
    }

    DcMotor getMotorDS() {
        return hardwareMap.dcMotor.get("MotorDS");
    }

    DcMotor getMotorSS() {
        return hardwareMap.dcMotor.get("MotorSS");
    }

    BNO055IMU getGyro() {
        return hardwareMap.get(BNO055IMU.class, "imu");
    }

    RevRoboticsCoreHexMotor getHexBase() { return hardwareMap.get(RevRoboticsCoreHexMotor.class, "hexBase"); }

}
