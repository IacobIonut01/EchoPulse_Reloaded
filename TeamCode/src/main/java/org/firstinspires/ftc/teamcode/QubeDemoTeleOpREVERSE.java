package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.internal.android.dx.ssa.DomFront;

import java.util.prefs.BackingStoreException;


@TeleOp(name = "Qube teleop REVERSE", group = "FTC")
public class QubeDemoTeleOpREVERSE extends LinearOpMode {
    DcMotor motorDF;
    DcMotor motorDS;
    DcMotor motorSF;
    DcMotor motorSS;

    @Override
    public void runOpMode() throws InterruptedException {
        Servo servo = hardwareMap.servo.get("servo") ;

        motorDF = hardwareMap.dcMotor.get("MotorDF");
        motorDS = hardwareMap.dcMotor.get("MotorDS");
        motorSF = hardwareMap.dcMotor.get("MotorSF");
        motorSS = hardwareMap.dcMotor.get("MotorSS");

        reversePolarity();

        motorDF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorDS.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorSF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorSS.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        double numarFrumos = 0.66;
        double acceleration = 0.1;
        double currentPowerY = 0;
        double currentPowerX = 0;
        double currentYawPower = 0;

        waitForStart();

        while (opModeIsActive()){

            //Move FORWARD/BACKWARD
            if(gamepad1.left_stick_y < 0) {
                if(currentPowerY <= numarFrumos)
                    currentPowerY += acceleration;

                motorDF.setPower(currentPowerY);
                motorDS.setPower(currentPowerY);
                motorSF.setPower(currentPowerY);
                motorSS.setPower(currentPowerY);
            }else
            if(gamepad1.left_stick_y > 0) {
                if(currentPowerY <= numarFrumos)
                    currentPowerY += acceleration;

                motorDF.setPower(-currentPowerY);
                motorDS.setPower(-currentPowerY);
                motorSF.setPower(-currentPowerY);
                motorSS.setPower(-currentPowerY);
            }else
                currentPowerY = 0;

            //Strafe LEFT/RIGHT
            if(gamepad1.left_stick_x > 0) {
                if(currentPowerX <= numarFrumos)
                    currentPowerX += acceleration;

                motorDF.setPower(-currentPowerX);
                motorDS.setPower(currentPowerX);
                motorSF.setPower(currentPowerX);
                motorSS.setPower(-currentPowerX);
            }else
            if(gamepad1.left_stick_x < 0) {
                if(currentPowerX <= numarFrumos)
                    currentPowerX += acceleration;

                motorDF.setPower(currentPowerX);
                motorDS.setPower(-currentPowerX);
                motorSF.setPower(-currentPowerX);
                motorSS.setPower(currentPowerX);
            } else
                currentPowerX = 0;

            //Rotate proba:fata-spate ++++/----
            if(gamepad1.right_stick_x > 0) {
                if(currentYawPower <= numarFrumos)
                    currentYawPower += acceleration;

                motorDF.setPower(-currentYawPower);
                motorDS.setPower(-currentYawPower);
                motorSF.setPower(currentYawPower);
                motorSS.setPower(currentYawPower);
            }else
            if(gamepad1.right_stick_x < 0) {
                if(currentYawPower <= numarFrumos)
                    currentYawPower += acceleration;

                motorDF.setPower(currentYawPower);
                motorDS.setPower(currentYawPower);
                motorSF.setPower(-currentYawPower);
                motorSS.setPower(-currentYawPower);
            }else
                currentYawPower = 0;

            //no movement
            if(gamepad1.left_stick_x == 0 && gamepad1.left_stick_y == 0 && gamepad1.right_stick_x == 0){
                motorDF.setPower(0);
                motorDS.setPower(0);
                motorSF.setPower(0);
                motorSS.setPower(0);
            }

            if(gamepad1.y){
                servo.setPosition(servo.getPosition() + 0.001);
            } else
            if(gamepad1.a)
                servo.setPosition(servo.getPosition() - 0.001);
        }
    }

    void reversePolarity(){
        motorDF.setDirection(DcMotorSimple.Direction.FORWARD);
        motorDS.setDirection(DcMotorSimple.Direction.FORWARD);
        motorSF.setDirection(DcMotorSimple.Direction.REVERSE);
        motorSS.setDirection(DcMotorSimple.Direction.REVERSE);
    }
}