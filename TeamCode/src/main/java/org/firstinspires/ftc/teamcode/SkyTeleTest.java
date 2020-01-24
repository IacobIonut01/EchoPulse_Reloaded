package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import static org.firstinspires.ftc.teamcode.Constants.Direction.HLEFT;
import static org.firstinspires.ftc.teamcode.Constants.Direction.HRIGHT;
import static org.firstinspires.ftc.teamcode.Constants.Direction.LEFT;
import static org.firstinspires.ftc.teamcode.Constants.Direction.RIGHT;

@TeleOp(name = "Sky Teleop Test")
public class SkyTeleTest extends LinearOpMode {

    private DcMotor motorSF, motorDF, motorDS, motorSS;
    private double speedDevider = 1;
    private boolean toggleSpeed = false;
    private double rotPower = 0.75;
    private double sasiuPowerX, sasiuPowerXR;

    @Override
    public void runOpMode() {

        EchoPulse_Parts parts = new EchoPulse_Parts(hardwareMap);
        motorSF = parts.getMotorSF();
        motorDF = parts.getMotorDF();
        motorDS = parts.getMotorDS();
        motorSS = parts.getMotorSS();
        motorSF.setDirection(DcMotor.Direction.FORWARD);
        motorDF.setDirection(DcMotor.Direction.FORWARD);
        motorDS.setDirection(DcMotor.Direction.FORWARD);
        motorSS.setDirection(DcMotor.Direction.FORWARD);

        motorDF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorSF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorSS.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorDS.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                double sasiuPowerY = (-gamepad1.left_stick_y) / speedDevider;
                sasiuPowerX = (gamepad1.left_stick_x) / speedDevider;
                sasiuPowerXR = (gamepad1.right_stick_x) / speedDevider;

                speedDevider();

                if (gamepad1.left_stick_y == 0) {
                    motorSF.setPower(0);
                    motorDF.setPower(0);
                    motorDS.setPower(0);
                    motorSS.setPower(0);
                }

                if (-gamepad1.left_stick_y > 0 || -gamepad1.left_stick_y < 0) {
                    motorSF.setPower(-sasiuPowerY);
                    motorDF.setPower(sasiuPowerY);
                    motorDS.setPower(sasiuPowerY);
                    motorSS.setPower(-sasiuPowerY);
                }

                if (gamepad1.left_stick_x < 0)
                    setSteering(HLEFT, false);

                if (gamepad1.left_stick_x > 0)
                    setSteering(HRIGHT, false);

                if (gamepad1.right_stick_x < 0) {
                    if (speedDevider == 1)
                        setSteering(LEFT, false);
                    else
                        setSteering(LEFT, true);
                }
                if (gamepad1.right_stick_x > 0) {
                    if (speedDevider == 1)
                        setSteering(RIGHT, false);
                    else
                        setSteering(RIGHT, true);
                }

            }
        }

    }

    private void setSteering(Constants.Direction direction, boolean useDivider) {
        switch (direction) {
            case LEFT:
                if (!useDivider) {
                    motorSF.setPower(sasiuPowerXR);
                    motorDF.setPower(sasiuPowerXR);
                    motorDS.setPower(sasiuPowerXR);
                    motorSS.setPower(sasiuPowerXR);
                } else {
                    motorSF.setPower(sasiuPowerXR / 1.25);
                    motorDF.setPower(sasiuPowerXR / 1.25);
                    motorDS.setPower(sasiuPowerXR / 1.25);
                    motorSS.setPower(sasiuPowerXR / 1.25);
                }
                break;
            case RIGHT:
                if (!useDivider) {
                    motorSF.setPower(-sasiuPowerXR);
                    motorDF.setPower(-sasiuPowerXR);
                    motorDS.setPower(-sasiuPowerXR);
                    motorSS.setPower(-sasiuPowerXR);
                } else {
                    motorSF.setPower(-sasiuPowerXR / 1.25);
                    motorDF.setPower(-sasiuPowerXR / 1.25);
                    motorDS.setPower(-sasiuPowerXR / 1.25);
                    motorSS.setPower(-sasiuPowerXR / 1.25);
                }
                break;
            case HLEFT:
                motorSF.setPower(sasiuPowerX);
                motorDF.setPower(sasiuPowerX);
                motorDS.setPower(-sasiuPowerX);
                motorSS.setPower(-sasiuPowerX);
                break;
            case HRIGHT:
                motorSF.setPower(-sasiuPowerX);
                motorDF.setPower(-sasiuPowerX);
                motorDS.setPower(sasiuPowerX);
                motorSS.setPower(sasiuPowerX);
                break;
        }
    }

    private void speedDevider() {
        if (gamepad1.a && toggleSpeed) {
            speedDevider = 2;
            rotPower = 0.5;
            toggleSpeed = false;
        } //SlowMode ON
        else if (gamepad1.a && !toggleSpeed) {
            speedDevider = 1;
            rotPower = 1;
            toggleSpeed = true;
        } //SlowMode OFF
    }
}
