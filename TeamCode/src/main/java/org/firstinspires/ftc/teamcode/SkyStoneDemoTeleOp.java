package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.motors.RevRoboticsCoreHexMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.List;

@TeleOp(name = "SkyStone TeleOp Demo")
public class SkyStoneDemoTeleOp extends LinearOpMode {

    private DcMotor motorDF;
    private DcMotor motorDS;
    private DcMotor motorSF;
    private DcMotor motorSS;

    //servo-urile de la sistemul de suctiune
    private Servo servo0;
    private Servo servo1;
    private Servo servo2;
    private Servo servo3;

    //servo-urile de la apucator
    private Servo ax0;
    private Servo ax1;

    //motorul bratului
    private DcMotor motoryes;


    @Override
    public void runOpMode() {
        motorDF = hardwareMap.dcMotor.get("MotorDF");
        motorDS = hardwareMap.dcMotor.get("MotorDS");
        motorSF = hardwareMap.dcMotor.get("MotorSF");
        motorSS = hardwareMap.dcMotor.get("MotorSS");

        servo0 = hardwareMap.servo.get("sugere0");
        servo1 = hardwareMap.servo.get("sugere1");
        servo2 = hardwareMap.servo.get("sugere2");
        servo3 = hardwareMap.servo.get("sugere3");

        ax0 = hardwareMap.servo.get("ax0");
        ax1 = hardwareMap.servo.get("ax1");

        motoryes = hardwareMap.dcMotor.get("bratan");

        //regleaza polaritatea motoarelor
        motorDF.setDirection(DcMotorSimple.Direction.REVERSE);
        motorDS.setDirection(DcMotorSimple.Direction.REVERSE);
        motorSF.setDirection(DcMotorSimple.Direction.FORWARD);
        motorSS.setDirection(DcMotorSimple.Direction.FORWARD);

        motorSS.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorDS.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorSF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorDF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        while (opModeIsActive()) {

            double x = -gamepad1.left_stick_x;
            double y = gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;
            mecanum(x, y, turn);

            //sistemul de suctiune
            if (gamepad1.a) {
                servo0.setPosition(0);
                servo2.setPosition(0);
                servo1.setPosition(1);
                servo3.setPosition(1);
            } else if (gamepad1.b) {
                servo0.setPosition(1);
                servo2.setPosition(1);
                servo1.setPosition(0);
                servo3.setPosition(0);
            } else {
                servo0.setPosition(0.5);
                servo2.setPosition(0.5);
                servo1.setPosition(0.5);
                servo3.setPosition(0.5);
            }

            telemetry.addData("servo0: ", servo0.getPosition());
            telemetry.addData("servo1: ", servo1.getPosition());
            telemetry.addData("servo2: ", servo2.getPosition());
            telemetry.addData("servo3: ", servo3.getPosition());
            telemetry.update();

            if (gamepad2.right_stick_y > 0)
                motoryes.setPower(gamepad2.right_stick_y * 0.8);
            else if (gamepad2.right_stick_y < 0)
                motoryes.setPower(gamepad2.right_stick_y * 0.2);
            else
                motoryes.setPower(0);

            telemetry.addData("motorYes: ", motoryes.getCurrentPosition());
            telemetry.update();


            if (gamepad2.a)
                ax0.setPosition(ax0.getPosition()+0.001);
            if (gamepad2.b)
                ax0.setPosition(ax0.getPosition()-0.001);
            if (gamepad2.x)
                ax1.setPosition(ax1.getPosition()+0.001);
            if (gamepad2.y)
                ax1.setPosition(ax1.getPosition()-0.001);

            telemetry.addData("ax0: ", ax0.getPosition());
            telemetry.addData("ax1: ", ax1.getPosition());
        }
    }

    /**
     * Control a mecanum drive base with three double inputs
     *
     * @param Strafe  is the first double X value which represents how the base should strafe
     * @param Forward is the only double Y value which represents how the base should drive forward
     * @param Turn    is the second double X value which represents how the base should turn
     */
    private void mecanum(double Strafe, double Forward, double Turn) {
        //Find the magnitude of the controller's input
        double r = Math.hypot(Strafe, Forward);

        //returns point from +X axis to point (forward, strafe)
        double robotAngle = Math.atan2(Forward, Strafe) - Math.PI / 4;

        //Quantity to turn by (turn)
        double rightX = Turn;

        //double vX represents the velocities sent to each motor
        final double v1 = (r * Math.cos(robotAngle)) + rightX;
        final double v2 = (r * Math.sin(robotAngle)) - rightX;
        final double v3 = (r * Math.sin(robotAngle)) + rightX;
        final double v4 = (r * Math.cos(robotAngle)) - rightX;

        motorSF.setPower(v1);
        motorDF.setPower(v2);
        motorSS.setPower(v3);
        motorDS.setPower(v4);
    }

}
