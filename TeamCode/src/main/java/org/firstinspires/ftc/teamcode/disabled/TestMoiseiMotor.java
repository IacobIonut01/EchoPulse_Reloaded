package org.firstinspires.ftc.teamcode.disabled;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
@TeleOp(name = "testulet")
public class TestMoiseiMotor extends LinearOpMode {
    private DcMotor motorel;
    private DcMotor motorel2;
    private Servo servorel;

    @Override
    public void runOpMode() throws InterruptedException {

        waitForStart();

        motorel = hardwareMap.dcMotor.get("motor");
        motorel2 = hardwareMap.dcMotor.get("motor1");
        servorel = hardwareMap.servo.get("servo");

        while(opModeIsActive()){
            if(gamepad1.left_stick_y == 0) {
                motorel.setPower(0);
                motorel2.setPower(0);
            }
            else{
                motorel.setPower(gamepad1.left_stick_y);
                motorel2.setPower(-gamepad1.left_stick_y);
            }

            if(gamepad1.y)
                servorel.setPosition(1);
            else if(gamepad1.a)
                servorel.setPosition(0);
        }
    }
}
