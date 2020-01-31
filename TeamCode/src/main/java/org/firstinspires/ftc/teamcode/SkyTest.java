package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import static org.firstinspires.ftc.teamcode.Constants.Direction.FORWARD;
import static org.firstinspires.ftc.teamcode.Constants.Direction.HLEFT;
import static org.firstinspires.ftc.teamcode.Constants.Direction.LEFT;
import static org.firstinspires.ftc.teamcode.Constants.Direction.RIGHT;

@Disabled
@Autonomous(name = "Sky Autonomous Test")
public class SkyTest extends LinearOpMode {

    private double countsPerRotation = 28;
    private double rotatiiMari = 40;
    private double diametruRoata = 10;
    private double precision = 1;
    private double countsPerCM = (countsPerRotation * rotatiiMari) / (Math.PI * diametruRoata) * precision;

    private DcMotor motorSF, motorDF, motorDS, motorSS;
    private BNO055IMU imu;
    private Orientation lastAngles = new Orientation();
    private double globalAngle = 0, power = 0.3;

    @Override
    public void runOpMode() {

        EchoPulse_Parts parts = new EchoPulse_Parts(hardwareMap);
        motorSF = parts.getMotorSF();
        motorDF = parts.getMotorDF();
        motorDS = parts.getMotorDS();
        motorSS = parts.getMotorSS();
        imu = parts.getGyro();

        BNO055IMU.Parameters parametersIMU = new BNO055IMU.Parameters();
        parametersIMU.mode = BNO055IMU.SensorMode.IMU;
        parametersIMU.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parametersIMU.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parametersIMU.loggingEnabled = false;

        motorSF.setDirection(DcMotor.Direction.REVERSE);
        motorDF.setDirection(DcMotor.Direction.REVERSE);
        motorDS.setDirection(DcMotor.Direction.REVERSE);
        motorSS.setDirection(DcMotor.Direction.REVERSE);

        motorDF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorSF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorSS.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorDS.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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
        while (opModeIsActive()) {

            telemetry.addData("Test","#001");
            gryoStatus();

            // Angle Rotation
            // > 0 Right
            // < 0 Left
            // -180 0 180
            rotate(60 /*angle*/);
            // Assisted movement
            // Gyro axes differences will automatically try to force the robot to move in straight line
            // Even if it collides
            move(FORWARD, 50);
            // Steering
            steer(LEFT, 50);
            // Horizontal Steering
            steer(HLEFT, 20);
        }

    }

    private void gryoStatus(){
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        Acceleration gravity = imu.getOverallAcceleration();

        double angleZ = angles.firstAngle;
        double angleY = angles.secondAngle;
        double angleX = angles.thirdAngle;

        double gravX = gravity.xAccel;
        double gravY = gravity.yAccel;
        double gravZ = gravity.zAccel;


        telemetry.addData("Z:", angleZ);
        telemetry.addData("Y:", angleY);
        telemetry.addData("X:", angleX);

        telemetry.addData("grav X", gravX);
        telemetry.addData("grav Y", gravY);
        telemetry.addData("grav Z", gravZ);

        telemetry.addData("Movement Assist Power", checkDirection());

        telemetry.update();
    }

    private void rotate(int degrees) {
        resetAngle();
        if (degrees < 0) {   // turn right.
            steer(LEFT, power);
        } else if (degrees > 0) {   // turn left.
            steer(RIGHT, power);
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

    private void move(Constants.Direction direction, int dist) {
        int countsNeeded = (int)countsPerCM * dist;

        motorDF.setTargetPosition(motorDF.getCurrentPosition() + countsNeeded);
        motorDS.setTargetPosition(motorDS.getCurrentPosition() + countsNeeded);
        motorSF.setTargetPosition(motorSF.getCurrentPosition() + countsNeeded);
        motorSS.setTargetPosition(motorSS.getCurrentPosition() + countsNeeded);

        motorDF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorDS.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorSF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorSS.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        resetAngle();
        switch (direction) {
            case FORWARD:
                motorSF.setPower(-power);
                motorSS.setPower(-power);
                motorDF.setPower(power - checkDirection());
                motorDS.setPower(power - checkDirection());
                break;
            case BACKWARD:
                motorSF.setPower(power);
                motorSS.setPower(power);
                motorDF.setPower(-power + checkDirection());
                motorDS.setPower(-power + checkDirection());
                break;
        }

        while(motorDF.isBusy() && motorDS.isBusy() && motorSF.isBusy() && motorSS.isBusy()){}

        stopAuto();

        motorDF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorDS.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorSF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorSS.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private void move(Constants.Direction direction, int dist , double powerMultiply) {
        int countsNeeded = (int)countsPerCM * dist;

        motorDF.setTargetPosition(motorDF.getCurrentPosition() + countsNeeded);
        motorDS.setTargetPosition(motorDS.getCurrentPosition() + countsNeeded);
        motorSF.setTargetPosition(motorSF.getCurrentPosition() + countsNeeded);
        motorSS.setTargetPosition(motorSS.getCurrentPosition() + countsNeeded);

        motorDF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorDS.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorSF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorSS.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        resetAngle();
        double multiplication = powerMultiply > 0 ? powerMultiply : 1;
        switch (direction) {
            case FORWARD:
                motorSF.setPower((-power) * multiplication);
                motorSS.setPower((-power) * multiplication);
                motorDF.setPower((power - checkDirection()) * multiplication);
                motorDS.setPower((power - checkDirection()) * multiplication);
                break;
            case BACKWARD:
                motorSF.setPower((power) * multiplication);
                motorSS.setPower((power) * multiplication);
                motorDF.setPower((-power + checkDirection()) * multiplication);
                motorDS.setPower((-power + checkDirection()) * multiplication);
                break;
        }

        while(motorDF.isBusy() && motorDS.isBusy() && motorSF.isBusy() && motorSS.isBusy()){}

        stopAuto();

        motorDF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorDS.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorSF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorSS.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private void steer(Constants.Direction direction, int dist) {
        int countsNeeded = (int)countsPerCM * dist;

        motorDF.setTargetPosition(motorDF.getCurrentPosition() - countsNeeded);
        motorDS.setTargetPosition(motorDS.getCurrentPosition() + countsNeeded);
        motorSF.setTargetPosition(motorSF.getCurrentPosition() + countsNeeded);
        motorSS.setTargetPosition(motorSS.getCurrentPosition() - countsNeeded);

        motorDF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorDS.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorSF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorSS.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        switch (direction) {
            case LEFT:
                motorSF.setPower(power);
                motorDF.setPower(power);
                motorDS.setPower(power);
                motorSS.setPower(power);
                break;
            case RIGHT:
                motorSF.setPower(-power);
                motorDF.setPower(-power);
                motorDS.setPower(-power);
                motorSS.setPower(-power);
                break;
            case HLEFT:
                motorSF.setPower(power);
                motorDF.setPower(power);
                motorDS.setPower(-power);
                motorSS.setPower(-power);
                break;
            case HRIGHT:
                motorSF.setPower(-power);
                motorDF.setPower(-power);
                motorDS.setPower(power);
                motorSS.setPower(power);
                break;
        }

        while(motorDF.isBusy() && motorDS.isBusy() && motorSF.isBusy() && motorSS.isBusy()){}

        stopAuto();

        motorDF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorDS.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorSF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorSS.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private void steer(Constants.Direction direction, int dist, double rotPower) {
        int countsNeeded = (int)countsPerCM * dist;

        motorDF.setTargetPosition(motorDF.getCurrentPosition() - countsNeeded);
        motorDS.setTargetPosition(motorDS.getCurrentPosition() + countsNeeded);
        motorSF.setTargetPosition(motorSF.getCurrentPosition() + countsNeeded);
        motorSS.setTargetPosition(motorSS.getCurrentPosition() - countsNeeded);

        motorDF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorDS.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorSF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorSS.setMode(DcMotor.RunMode.RUN_TO_POSITION);
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

        while(motorDF.isBusy() && motorDS.isBusy() && motorSF.isBusy() && motorSS.isBusy()){}

        stopAuto();

        motorDF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorDS.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorSF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorSS.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private void steer(Constants.Direction direction) {
        switch (direction) {
            case LEFT:
                motorSF.setPower(power);
                motorDF.setPower(power);
                motorDS.setPower(power);
                motorSS.setPower(power);
                break;
            case RIGHT:
                motorSF.setPower(-power);
                motorDF.setPower(-power);
                motorDS.setPower(-power);
                motorSS.setPower(-power);
                break;
            case HLEFT:
                motorSF.setPower(power);
                motorDF.setPower(power);
                motorDS.setPower(-power);
                motorSS.setPower(-power);
                break;
            case HRIGHT:
                motorSF.setPower(-power);
                motorDF.setPower(-power);
                motorDS.setPower(power);
                motorSS.setPower(power);
                break;
        }
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

    /*
     * System Management
     **/

    private void waitAndStop(int milliseconds) {
        sleep(milliseconds);
        stopAuto();
    }

    private void stopAuto() {
        motorDS.setPower(0);
        motorDF.setPower(0);
        motorSS.setPower(0);
        motorSF.setPower(0);
    }

    private void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
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
}
