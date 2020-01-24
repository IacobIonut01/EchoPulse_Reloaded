package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
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

import static org.firstinspires.ftc.teamcode.Constants.Direction.LEFT;
import static org.firstinspires.ftc.teamcode.Constants.Direction.RIGHT;

@Autonomous(name = "How fast is the Autonomous period? YES", group = "FTC")
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
    private double globalAngle = 0;
    private Orientation lastAngles = new Orientation();

    @Override
    public void runOpMode() throws InterruptedException {

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parametersIMU = new BNO055IMU.Parameters();
        parametersIMU.mode = BNO055IMU.SensorMode.IMU;
        parametersIMU.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parametersIMU.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parametersIMU.loggingEnabled = false;

        motorDF = hardwareMap.dcMotor.get("MotorDF");
        motorDS = hardwareMap.dcMotor.get("MotorDS");
        motorSF = hardwareMap.dcMotor.get("MotorSF");
        motorSS = hardwareMap.dcMotor.get("MotorSS");

        motorSS.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorDS.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorSF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorDF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        reversePolarity();

        imu.initialize(parametersIMU);

        //CALIBRARE GYRO START
        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("IMU Calibration Status :", imu.getCalibrationStatus().toString());
        telemetry.update();

        waitForStart();

        while (opModeIsActive()){
            DoAutonomusStuff(this.didFunctionRun);
        }

    }

    private void DoAutonomusStuff(boolean didFunctionRun){
        if(!didFunctionRun){
            //moveTo(80, 0.66);
            moveTo(62, 0.69);
            strafeTo(76.2, 0.69);
            strafeTo(-203.2, 0.69);
            strafeTo( 94, 0.69);
            rotate(180);
            this.didFunctionRun = true;
        }
    }

    void moveTo(double dist, double speed){
        int countsNeeded = (int)(countsPerCM * dist);

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

    void strafeTo(double dist, double speed){
        int countsNeeded = (int)(countsPerCM * dist);

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

    private void rotate(int degrees) {
        resetAngle();
        if (degrees < 0) {   // turn right.
            steer(LEFT, 0.66);
        } else if (degrees > 0) {   // turn left.
            steer(RIGHT, 0.66);
        } else return;
        if (degrees < 0) {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0) {}
            while (opModeIsActive() && getAngle() > degrees) {}
        } else    // left turn.
            while (opModeIsActive() && getAngle() < degrees) {}
        stopAuto();
        sleep(1000);
        resetAngle();
    }

    private void steer(Constants.Direction direction, double rotPower) {
        switch (direction) {
            case LEFT:
                motorSF.setPower(rotPower);
                motorDF.setPower(rotPower);
                motorDS.setPower(rotPower);
                motorSS.setPower(rotPower);
                break;
            case RIGHT:
                motorSF.setPower(-rotPower);
                motorDF.setPower(-rotPower);
                motorDS.setPower(-rotPower);
                motorSS.setPower(-rotPower);
                break;
            case HLEFT:
                motorSF.setPower(rotPower);
                motorDF.setPower(rotPower);
                motorDS.setPower(-rotPower);
                motorSS.setPower(-rotPower);
                break;
            case HRIGHT:
                motorSF.setPower(-rotPower);
                motorDF.setPower(-rotPower);
                motorDS.setPower(rotPower);
                motorSS.setPower(rotPower);
                break;
        }
    }

    private double getAngle() {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    private void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    private void stopAuto() {
        motorDS.setPower(0);
        motorDF.setPower(0);
        motorSS.setPower(0);
        motorSF.setPower(0);
    }

    void reversePolarity(){
        motorDF.setDirection(DcMotorSimple.Direction.FORWARD);
        motorDS.setDirection(DcMotorSimple.Direction.FORWARD);
        motorSF.setDirection(DcMotorSimple.Direction.REVERSE);
        motorSS.setDirection(DcMotorSimple.Direction.REVERSE);
    }

}