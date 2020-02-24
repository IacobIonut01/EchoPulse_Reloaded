package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Calibrare SERVO")
public class CalibrareServo extends LinearOpMode {

    @Override
    public void runOpMode() {
        Servo p0 = hardwareMap.servo.get("servo0");
        Servo p1 = hardwareMap.servo.get("servo1");

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.a) {
                p0.setPosition(0);
                p1.setPosition(0);
            }
        }
    }

}
