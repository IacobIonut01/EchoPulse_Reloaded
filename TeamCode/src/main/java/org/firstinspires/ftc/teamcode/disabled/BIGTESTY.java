package org.firstinspires.ftc.teamcode.disabled;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class BIGTESTY extends LinearOpMode {

    @Override
    public void runOpMode() {
        DcMotor dctest = hardwareMap.dcMotor.get("dcmotor");
        DcMotor dctest1 = hardwareMap.dcMotor.get("dcmotor1");

        dctest.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {
            dctest.setPower(gamepad1.left_stick_y);
            dctest1.setPower(gamepad1.left_stick_y);
        }
    }
}
