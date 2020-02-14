package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;
import java.util.Locale;

import static org.firstinspires.ftc.teamcode.Constants.Direction.LEFT;
import static org.firstinspires.ftc.teamcode.Constants.Direction.RIGHT;

@Autonomous(name = "Autonoma -<<", group = "FTC")
public class FUCK extends LinearOpMode {

    private BNO055IMU imu;

    private double countsPerRotation = 28;
    private double rotatiiMari = 40;
    private double diametruRoata = 10;
    private double precision = 1;
    private double countsPerCM = (countsPerRotation * rotatiiMari) / (Math.PI * diametruRoata) * precision;
    private DcMotor motorDF;
    private DcMotor motorDS;
    private DcMotor motorSF;
    private DcMotor motorSS;
    private boolean didFunctionRun = false;
    private double globalAngle = 0;
    private int skystonePos = 0;
    private Orientation lastAngles = new Orientation();

    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";

    private static final String VUFORIA_KEY = "AexRVhH/////AAABmRwlvZsGx0Oor/vwJ7jQe7w0CCm9dj4XqZzZM+GKL0bAOBWbJZCukHVq80UOiV4X6fZipT53Y/ekerVZ4Y73NnXBy3fxFkz11J6LweNoe5HZNQEXbeCuTGGc4XhidpQPDhXGjwQW302VtF6gK4z9Sru7Lqyu+eYSeSfy8UhVs2VYLlCuP8vO8gJCbFG8dptNQGn/NVZP7BTugsioepH2DnoKmkj1kwMdbiQGZkAOLYrI/RqPVdR1qOyqY2dX4s2N3LPWkN39fh6VVMm7A353UAE4OYDPgj9Id4wWBlKUL0inI5TgbMFRTkPcvykUDS1N29aZ6tmBfIixe/RRWQXh4WAteCeZ34wMnL/bts8EDy4g";

    private VuforiaLocalizer vuforia;

    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {

        /*
        initVuforia();
        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }*/
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
        /*
        if (tfod != null) {
            tfod.activate();
        }*/

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("IMU Calibration Status :", imu.getCalibrationStatus().toString());
        telemetry.update();

        waitForStart();

        /*
        while (opModeIsActive()){
            DoAutonomusStuff(this.didFunctionRun);
        }*/

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                DoAutonomusStuff(didFunctionRun);
            }
        }

        if (tfod != null) {
            tfod.shutdown();
        }
    }

    private void DoAutonomusStuff(boolean didFunctionRun){
        if(!didFunctionRun){
            moveTo(-25, 0.2);
            this.didFunctionRun = true;
        }
    }

    private void skystoneFinder() {
        boolean skystoneFound = false;
        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                telemetry.addData("# Object Detected", updatedRecognitions.size());
                // step through the list of recognitions and display boundary info.
                for (int i = 0; i < updatedRecognitions.size(); i++) {
                    Recognition recognition = updatedRecognitions.get(i);
                    telemetry.addData(String.format(Locale.ENGLISH, "label (%d)", i), recognition.getLabel());
                    telemetry.addData(String.format(Locale.ENGLISH, "  left,top (%d)", i), "%.03f , %.03f",
                            recognition.getLeft(), recognition.getTop());
                    telemetry.addData(String.format(Locale.ENGLISH, "  right,bottom (%d)", i), "%.03f , %.03f",
                            recognition.getRight(), recognition.getBottom());
                    if (recognition.getLabel().equals("Skystone")) {
                        skystoneFound = true;
                    }
                }
                telemetry.update();
                while (!skystoneFound) {
                    strafeTo(-23.706, 0.2);
                    skystonePos++;
                    if (skystonePos > 2)
                        skystoneFound=true;
                }
                if (skystoneFound) {
                    stopAuto();
                    moveTo(10, 0.3);
                    moveTo(-10, 0.3);
                }
            }
        }
    }

    private double checkDirection() {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .10;

        angle = getAngle();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.\
        correction *= gain;

        return correction;
    }

    private void moveTo(double cm, double speed){
        int countsNeeded = (int)(countsPerCM * cm);

        motorDF.setTargetPosition(motorDF.getCurrentPosition() - countsNeeded);
        motorDS.setTargetPosition(motorDS.getCurrentPosition() - countsNeeded);
        motorSF.setTargetPosition(motorSF.getCurrentPosition() - countsNeeded);
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

    private void strafeTo(double cm, double speed){
        int countsNeeded = (int)(countsPerCM * cm);

        motorDF.setTargetPosition(motorDF.getCurrentPosition() + countsNeeded);
        motorDS.setTargetPosition(motorDS.getCurrentPosition() - countsNeeded);
        motorSF.setTargetPosition(motorSF.getCurrentPosition() - countsNeeded);
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

    private void reversePolarity(){
        motorDF.setDirection(DcMotorSimple.Direction.FORWARD);
        motorDS.setDirection(DcMotorSimple.Direction.FORWARD);
        motorSF.setDirection(DcMotorSimple.Direction.REVERSE);
        motorSS.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.8;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

}