package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name = "Autonoma Qube REVERSE", group = "FTC")
public class QubeDemoREVERSE extends LinearOpMode {


    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;
    double countsPerRotation = 28;
    double rotatiiMari = 40;
    double diametruRoata = 10;
    double precision = 1;
    double countsPerCM = (countsPerRotation * rotatiiMari) / (Math.PI * diametruRoata) * precision;
    DcMotor motorDF;
    DcMotor motorDS;
    DcMotor motorSF;
    DcMotor motorSS;
    Servo servo;
    boolean didFunctionRun = false;

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
        reversePolarity();

        waitForStart();

        while (opModeIsActive()){
            DoAutonomusStuff(this.didFunctionRun);
        }

    }

    private void DoAutonomusStuff(boolean didFunctionRun){
        if(!didFunctionRun){
            //moveTo(80, 0.66);
            moveTo(70, 0.69);
            strafeTo(30, 0.69);
            moveTo(-70, 0.69);
            strafeTo(-30, 0.69);
            //servo.setPosition(0.85);
            this.didFunctionRun = true;
        }
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

    void reversePolarity(){
        motorDF.setDirection(DcMotorSimple.Direction.FORWARD);
        motorDS.setDirection(DcMotorSimple.Direction.FORWARD);
        motorSF.setDirection(DcMotorSimple.Direction.REVERSE);
        motorSS.setDirection(DcMotorSimple.Direction.REVERSE);
    }

}