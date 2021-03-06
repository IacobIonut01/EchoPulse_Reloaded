package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
@Disabled
@TeleOp(name = "Robot")
public class TeleOpWithDistSens extends LinearOpMode {
    private DcMotor motorDF;
    private DcMotor motorDS;
    private DcMotor motorSF;
    private DcMotor motorSS;

    private DistanceSensor sensorStg;
    private DistanceSensor sensorDr;

    @Override
    public void runOpMode() {
        sensorStg = hardwareMap.get(DistanceSensor.class, "senzor stg");
        sensorDr = hardwareMap.get(DistanceSensor.class, "senzor dr");

        DcMotor dctest = hardwareMap.dcMotor.get("dcmotor");
        DcMotor dctest1 = hardwareMap.dcMotor.get("dcmotor1");

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
            double turn = -gamepad1.right_stick_x ;
            mecanum(x, y, turn);

            dctest.setPower(gamepad1.left_trigger);
            dctest1.setPower(gamepad1.left_trigger);

            dctest.setPower(-gamepad1.right_trigger);
            dctest1.setPower(-gamepad1.right_trigger);

            telemetry.addData("Nu este necesara calibrarea", "");
            telemetry.addData("senzor stg: ", sensorStg.getDistance(DistanceUnit.CM));
            telemetry.addData("senzor dr: ", sensorDr.getDistance(DistanceUnit.CM));
            telemetry.update();

            double distStanga =  sensorStg.getDistance(DistanceUnit.CM);
            double distDreapta =  sensorDr.getDistance(DistanceUnit.CM);


            if(gamepad1.a){
                if (distDreapta > distStanga) {
                    while (check(distDreapta, distStanga)) {
                        motorSF.setPower(0.69);
                        motorSS.setPower(0.69);
                        telemetry.addData("Calibrare...", "");
                        telemetry.addData("senzor stg: ", distStanga);
                        telemetry.addData("senzor dr: ", distDreapta);
                        telemetry.update();
                    }
                    motorDF.setPower(0);
                    motorDS.setPower(0);
                }

                if (distStanga > distDreapta) {
                    while (check(distStanga, distDreapta)) {
                        motorDF.setPower(0.69);
                        motorDS.setPower(0.69);
                        telemetry.addData("Calibrare...", "");
                        telemetry.addData("senzor stg: ", distStanga);
                        telemetry.addData("senzor dr: ", distDreapta);
                        telemetry.update();
                    }
                    motorSF.setPower(0);
                    motorSS.setPower(0);
                }
            }
        }
    }

    private boolean check(double first, double second) {
        return Math.abs(first - second) <= 1;
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
