package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.motors.RevRoboticsCoreHexMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.List;

@TeleOp(name = "Ciordeala ieftina+")
public class SkyStoneOp extends LinearOpMode {

    private DcMotor motorDF;
    private DcMotor motorDS;
    private DcMotor motorSF;
    private DcMotor motorSS;

    @Override
    public void runOpMode() {
        motorDF = hardwareMap.dcMotor.get("MotorDF");
        motorDS = hardwareMap.dcMotor.get("MotorDS");
        motorSF = hardwareMap.dcMotor.get("MotorSF");
        motorSS = hardwareMap.dcMotor.get("MotorSS");
        Servo servo0 = hardwareMap.servo.get("sugere0");
        Servo servo1 = hardwareMap.servo.get("sugere1");
        Servo servo2 = hardwareMap.servo.get("sugere2");
        Servo servo3 = hardwareMap.servo.get("sugere3");
        Servo ax0 = hardwareMap.servo.get("ax0");
        Servo ax1 = hardwareMap.servo.get("ax1");
        DcMotor motoryes = hardwareMap.dcMotor.get("bratan");

        motorDF.setDirection(DcMotorSimple.Direction.FORWARD);
        motorDS.setDirection(DcMotorSimple.Direction.FORWARD);
        motorSF.setDirection(DcMotorSimple.Direction.REVERSE);
        motorSS.setDirection(DcMotorSimple.Direction.REVERSE);

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

            telemetry.addData("Yes", motoryes.getCurrentPosition());
            telemetry.update();

            if (gamepad2.right_stick_y > 0)
                motoryes.setPower(gamepad2.right_stick_y * 0.8);
            else if (gamepad2.right_stick_y < 0)
                motoryes.setPower(gamepad2.right_stick_y * 0.2);
            else
                motoryes.setPower(0);

            if (gamepad2.a)
                ax0.setPosition(ax0.getPosition()+0.01);
            if (gamepad2.b)
                ax0.setPosition(ax0.getPosition()-0.01);
            if (gamepad2.x)
                ax1.setPosition(ax1.getPosition()+0.01);
            if (gamepad2.y)
                ax1.setPosition(ax1.getPosition()-0.01);
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
