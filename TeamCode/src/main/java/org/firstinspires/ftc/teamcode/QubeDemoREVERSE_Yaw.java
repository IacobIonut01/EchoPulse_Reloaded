package org.firstinspires.ftc.teamcode;

import android.graphics.drawable.GradientDrawable;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name = "Autonoma Qube REVERSE Yaw", group = "FTC")
public class QubeDemoREVERSE_Yaw extends LinearOpMode {


    double countsPerRotation = 28;
    double rotatiiMari = 40;
    double diametruRoata = 10;
    double precision = 1;
    double countsPerCM = (countsPerRotation * rotatiiMari) / (Math.PI * diametruRoata) * precision;
    DcMotor motorDF;
    DcMotor motorDS;
    DcMotor motorSF;
    DcMotor motorSS;
    private Servo servo;
    boolean didFunctionRun = false;
    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;

    double angleX, angleY, angleZ;
    double gravX,gravY,gravZ;

    public Servo getServo(){
        return servo;
    }

    private void setServo(Servo Servo){
        this.servo = Servo;
    }

    @Override
    public void runOpMode() throws InterruptedException {

        motorDF = hardwareMap.dcMotor.get("MotorDF");
        motorDS = hardwareMap.dcMotor.get("MotorDS");
        motorSF = hardwareMap.dcMotor.get("MotorSF");
        motorSS = hardwareMap.dcMotor.get("MotorSS");

        motorSS.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorDS.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorSF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorDF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        servo = hardwareMap.servo.get("servo");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        //parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        reversePolarity();

        waitForStart();

        while (opModeIsActive()){
            //DoAutonomusStuff(this.didFunctionRun);
            /*moveTo(100, 0.66);
            //moveTo(-15, 0.66);
            servo.setPosition(servo.getPosition() + 0.9);
            wait(100000);
            break;*/
            telemetryGyro();
        }

    }

    private void DoAutonomusStuff(boolean didFunctionRun){
        if(!didFunctionRun){
            //turn(90, 0.69);
            telemetryGyro();
        }
    }

    void telemetryGyro(){
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        gravity = imu.getOverallAcceleration();

        angleZ = angles.firstAngle;
        angleY = angles.secondAngle;
        angleX = angles.thirdAngle;

        gravX = gravity.xAccel;
        gravY = gravity.yAccel;
        gravZ = gravity.zAccel;


        telemetry.addData("Z:", angleZ);
        telemetry.addData("Y", angleY);
        telemetry.addData("X", angleX);

        telemetry.addData("grav X", gravX);
        telemetry.addData("grav Y", gravY);
        telemetry.addData("grav Z", gravZ);

        telemetry.update();
    }
    void moveTo(int dist, double speed){
        int countsNeeded = (int)countsPerCM * dist;

        motorDF.setTargetPosition(motorDF.getCurrentPosition() + countsNeeded);
        motorDS.setTargetPosition(motorDS.getCurrentPosition() + countsNeeded);
        motorSF.setTargetPosition(motorSF.getCurrentPosition() + countsNeeded);
        motorSS.setTargetPosition(motorSS.getCurrentPosition() + countsNeeded);

        motorDF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorDS.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorSF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorSS.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorDF.setPower(speed);
        motorDS.setPower(speed);
        motorSF.setPower(speed);
        motorSS.setPower(speed);


        while(motorDF.isBusy() && motorDS.isBusy() && motorSF.isBusy() && motorSS.isBusy()){
            int currentEncoderValue = motorDF.getCurrentPosition();
            telemetry.addData("motorDF encoder:", currentEncoderValue);
            telemetry.update();
        }

        motorDF.setPower(0);
        motorDS.setPower(0);
        motorSF.setPower(0);
        motorSS.setPower(0);

        motorDF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorDS.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorSF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorSS.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    void strafeTo(int dist, double speed){
        int countsNeeded = (int)countsPerCM * dist;

        motorDF.setTargetPosition(motorDF.getCurrentPosition() - countsNeeded);
        motorDS.setTargetPosition(motorDS.getCurrentPosition() + countsNeeded);
        motorSF.setTargetPosition(motorSF.getCurrentPosition() + countsNeeded);
        motorSS.setTargetPosition(motorSS.getCurrentPosition() - countsNeeded);

        motorDF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorDS.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorSF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorSS.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorDF.setPower(speed);
        motorDS.setPower(speed);
        motorSF.setPower(speed);
        motorSS.setPower(speed);


        while(motorDF.isBusy() && motorDS.isBusy() && motorSF.isBusy() && motorSS.isBusy()){
            int currentEncoderValue = motorDF.getCurrentPosition();
            telemetry.addData("motorDF encoder:", currentEncoderValue);
            telemetry.update();
        }

        motorDF.setPower(0);
        motorDS.setPower(0);
        motorSF.setPower(0);
        motorSS.setPower(0);

        motorDF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorDS.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorSF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorSS.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    void turn(int degrees, double speed){
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double startZangle = angles.firstAngle;
        double currentAngle = angles.firstAngle;
        while(currentAngle < startZangle + degrees){
            currentAngle = angles.firstAngle;

            telemetry.addData("Z angle: ", currentAngle);
            telemetry.update();

            motorDF.setPower(speed);
            motorDS.setPower(speed);
            motorSF.setPower(-speed);
            motorSS.setPower(- speed);
        }
        motorDF.setPower(0);
        motorDS.setPower(0);
        motorSF.setPower(0);
        motorSS.setPower(0);

    }
    void reversePolarity(){
        motorDF.setDirection(DcMotorSimple.Direction.FORWARD);
        motorDS.setDirection(DcMotorSimple.Direction.FORWARD);
        motorSF.setDirection(DcMotorSimple.Direction.REVERSE);
        motorSS.setDirection(DcMotorSimple.Direction.REVERSE);
    }
}
