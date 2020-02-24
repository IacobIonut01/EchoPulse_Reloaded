package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "SkyTeleop")
public class SkyTeleop extends LinearOpMode {

    private DcMotor motorDF;
    private DcMotor motorDS;
    private DcMotor motorSF;
    private DcMotor motorSS;

    @Override
    public void runOpMode() {
        DcMotor dctest = hardwareMap.dcMotor.get("dcmotor");
        DcMotor dctest1 = hardwareMap.dcMotor.get("dcmotor1");


        Servo platforma0 = hardwareMap.servo.get("servo0");
        Servo platforma1 = hardwareMap.servo.get("servo1");

        dctest.setDirection(DcMotorSimple.Direction.REVERSE);
        motorDF = hardwareMap.dcMotor.get("MotorDF");
        motorDS = hardwareMap.dcMotor.get("MotorDS");
        motorSF = hardwareMap.dcMotor.get("MotorSF");
        motorSS = hardwareMap.dcMotor.get("MotorSS");

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
            double turn = -gamepad1.right_stick_x;
            mecanum(x, y, turn);

            dctest.setPower(gamepad1.left_trigger);
            dctest1.setPower(gamepad1.left_trigger);

            dctest.setPower(-gamepad1.right_trigger);
            dctest1.setPower(-gamepad1.right_trigger);

            if (gamepad1.right_bumper) {
                platforma0.setPosition(0.55);
                platforma1.setPosition(0.55);
            }
            if (gamepad1.left_bumper) {
                platforma0.setPosition(1);
                platforma1.setPosition(0);
            }
        }


    }

    /**
     * Control a mecanum drive base with three double inputs
     *
     * @param Strafe  is the first double X value which represents how the base should strafe
     * @param Forward is the only double Y value which represents how the base should drive forward
     * @param Turn    is the second double X value which represents how the base should turn
     */
    public void mecanum(double Strafe, double Forward, double Turn) {
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
