package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
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

            /*
            //La <0.05 robotul mai ca nu vrea sa porneasca
            if(x < 0.05 || x > -0.05) x = 0;
            if(y < 0.05 || y > -0.05) y = 0;

            //Logarithmic control
            x = (x*10)*(x*10)*(x*10)/1000;
            y = (y*10)*(y*10)*(y*10)/1000;*/
            mecanum(x, y, turn);

            dctest.setPower(gamepad1.left_trigger);
            dctest1.setPower(gamepad1.left_trigger);

            dctest.setPower(-gamepad1.right_trigger);
            dctest1.setPower(-gamepad1.right_trigger);

            telemetry.addData("senzor stg: ", sensorStg.getDistance(DistanceUnit.CM));
            telemetry.addData("senzor dr: ", sensorDr.getDistance(DistanceUnit.CM));
            telemetry.update();

            boolean lock = false;
            if(gamepad1.a){
                if(sensorDr.getDistance(DistanceUnit.CM) > sensorStg.getDistance(DistanceUnit.CM)){
                    double dist = sensorDr.getDistance(DistanceUnit.CM);
                    if (!lock) dist = sensorStg.getDistance(DistanceUnit.CM);
                    lock=true;
                    while(sensorDr.getDistance(DistanceUnit.CM) > dist){
                        motorDF.setPower(0.6);
                        motorDF.setPower(0.6);

                        telemetry.addData("Corectie","");
                        telemetry.addData("senzor stg: ", sensorStg.getDistance(DistanceUnit.CM));
                        telemetry.addData("senzor dr: ", sensorDr.getDistance(DistanceUnit.CM));
                        telemetry.update();
                    }
                    motorDF.setPower(0);
                    motorDS.setPower(0);
                    lock=false;
                }else{
                    double dist = sensorStg.getDistance(DistanceUnit.CM);
                    if(!lock) dist = sensorDr.getDistance(DistanceUnit.CM);
                    lock=true;

                    while(sensorStg.getDistance(DistanceUnit.CM) > dist){
                        motorSF.setPower(0.6);
                        motorSF.setPower(0.6);

                        telemetry.addData("Corectie","");
                        telemetry.addData("senzor stg: ", sensorStg.getDistance(DistanceUnit.CM));
                        telemetry.addData("senzor dr: ", sensorDr.getDistance(DistanceUnit.CM));
                        telemetry.update();
                    }
                    motorSF.setPower(0);
                    motorSS.setPower(0);
                    lock=false;
                }

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
