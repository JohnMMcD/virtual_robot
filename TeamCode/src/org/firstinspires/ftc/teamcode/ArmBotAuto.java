package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * Example autonomous op mode by John M. Demonstrates use of gyro, color sensor, encoders, and telemetry.
 *
 */
@SuppressWarnings({"unused", "WeakerAccess"})
@TeleOp(name = "arm bot auto", group = "ArmBot")
public class ArmBotAuto extends LinearOpMode {

    private int BLUE_LINE_DETECTION = 200;
    private int REVERSE_90_ENCODER = 6000;
    private int MOVE_TO_CORNER = 8000;
    private ElapsedTime etLocal = new ElapsedTime();
    private ElapsedTime etOpMode = new ElapsedTime();

    private String getDistance(DistanceSensor sensor)
    {
        String sTempDistance;
        double dRawDistance = sensor.getDistance(DistanceUnit.CM);
        sTempDistance = dRawDistance < 815 ?  String.format("%f", dRawDistance) : "Undefined";
        return sTempDistance;
    }

    public void runOpMode(){
        telemetry.addData("Hardware mapping","begin");
        telemetry.update();
        DcMotor mtrBackLeft = hardwareMap.dcMotor.get("back_left_motor");
        DcMotor mtrFrontLeft = hardwareMap.dcMotor.get("front_left_motor");
        DcMotor mtrFrontRight = hardwareMap.dcMotor.get("front_right_motor");
        DcMotor mtrBackRight = hardwareMap.dcMotor.get("back_right_motor");
        mtrBackLeft.setDirection(DcMotor.Direction.REVERSE);
        mtrFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        mtrBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mtrFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mtrFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mtrBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        mtrBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mtrFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mtrFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mtrBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        DcMotor arm = hardwareMap.dcMotor.get("arm_motor");
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //GyroSensor gyro = hardwareMap.gyroSensor.get("gyro_sensor");
        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        Servo handServo = hardwareMap.servo.get("hand_servo");
        DistanceSensor frontDistance = hardwareMap.get(DistanceSensor.class, "front_distance");
        DistanceSensor leftDistance = hardwareMap.get(DistanceSensor.class, "left_distance");
        DistanceSensor rightDistance = hardwareMap.get(DistanceSensor.class, "right_distance");
        DistanceSensor backDistance = hardwareMap.get(DistanceSensor.class, "back_distance");
        telemetry.addData("Hardware mapping","end");
        telemetry.update();
        //gyro.init();

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.accelerationIntegrationAlgorithm = null;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationData = null;
        parameters.calibrationDataFile = "";
        parameters.loggingEnabled = false;
        parameters.loggingTag = "Who cares.";

        imu.initialize(parameters);

        ColorSensor colorSensor = hardwareMap.colorSensor.get("color_sensor");
        telemetry.addData("Press Start Button When Ready","");
        telemetry.addData("Front Distance", getDistance(frontDistance));
        telemetry.addData("Left Distance", getDistance(leftDistance));
        telemetry.addData("Right Distance", getDistance(rightDistance));
        telemetry.addData("Back Distance", getDistance(backDistance));
        telemetry.addData("Encoders"," %d %d %d %d", mtrBackLeft.getCurrentPosition(), mtrFrontLeft.getCurrentPosition(),
                mtrFrontRight.getCurrentPosition(), mtrBackRight.getCurrentPosition());
        telemetry.addData("Runtime",  etLocal.seconds());
        telemetry.update();

        gamepad1.setJoystickDeadzone(0.05f);

        waitForStart();
        etOpMode.reset();
        etLocal.reset();
        boolean bButtonWasPressed = false;
        int iState = 0;

        while (opModeIsActive()){
            double p1;
            double p2;
            double p3;
            double p4;
            telemetry.addData("Color","R %d  G %d  B %d", colorSensor.red(), colorSensor.green(), colorSensor.blue());
            //telemetry.addData("Heading"," %.1f", gyro.getHeading());
            Orientation orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
            double dDegrees = orientation.firstAngle * 180.0 / Math.PI;
            telemetry.addData("Heading", " %.1f", dDegrees);
            telemetry.addData("State", iState);
            telemetry.addData("Front Distance", getDistance(frontDistance));
            telemetry.addData("Left Distance", getDistance(leftDistance));
            telemetry.addData("Right Distance", getDistance(rightDistance));
            telemetry.addData("Back Distance", getDistance(backDistance));
            telemetry.addData("Encoders"," %d %d %d %d", mtrBackLeft.getCurrentPosition(), mtrFrontLeft.getCurrentPosition(),
                    mtrFrontRight.getCurrentPosition(), mtrBackRight.getCurrentPosition());
            telemetry.addData("Local Runtime",  etLocal.seconds());
            telemetry.addData("Op Mode Runtime",  etOpMode.seconds());
            telemetry.update();

            if (iState == 0) // Strafe left until the blue line is detected or distance is reached, then stop and advance the state
            {
                if (colorSensor.blue() < BLUE_LINE_DETECTION &&  leftDistance.getDistance(DistanceUnit.CM) > 10.0)
                {
                    mtrFrontLeft.setPower(-1.0);
                    mtrBackLeft.setPower(1.0);
                    mtrFrontRight.setPower(1.0);
                    mtrBackRight.setPower(-1.0);
                }
                else
                {
                    mtrFrontLeft.setPower(0);
                    mtrBackLeft.setPower(0);
                    mtrFrontRight.setPower(0);
                    mtrBackRight.setPower(0);
                    etLocal.reset();
                    iState++;
                }
            }
            else if (iState == 1) // blue line was detected; advance forward until the blue line is no longer detected
            {
                if (colorSensor.blue() > BLUE_LINE_DETECTION)
                {
                    mtrFrontLeft.setPower(1.0);
                    mtrBackLeft.setPower(1.0);
                    mtrFrontRight.setPower(1.0);
                    mtrBackRight.setPower(1.0);
                }
                else // blue line no longer detected; advance the state, but no need to stop
                {
                    iState++;
                }
            } // advancing forward until off the blue line
            else if (iState == 2) // moving forward to acquire the platform
            {
                if (colorSensor.blue() < BLUE_LINE_DETECTION)
                {
                    mtrFrontLeft.setPower(1.0);
                    mtrBackLeft.setPower(1.0);
                    mtrFrontRight.setPower(1.0);
                    mtrBackRight.setPower(1.0);
                }
                else
                {
                    mtrFrontLeft.setPower(0);
                    mtrBackLeft.setPower(0);
                    mtrFrontRight.setPower(0);
                    mtrBackRight.setPower(0);
                    mtrBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    mtrFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    mtrFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    mtrBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    mtrBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    mtrFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    mtrFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    mtrBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    etLocal.reset();
                    iState++;
                }
            } // moving forward to get platform
            else if (iState == 3) // grab the platform and turn
            {
                // TODO: lower servo
                mtrFrontLeft.setPower(-0.90);
                mtrBackLeft.setPower(-0.90);
                mtrFrontRight.setPower(-0.10);
                mtrBackRight.setPower(-0.10);
                // Check an encoder to make sure you don't go too far
                if (mtrFrontLeft.getCurrentPosition() < -REVERSE_90_ENCODER)
                {
                    mtrFrontLeft.setPower(0);
                    mtrBackLeft.setPower(0);
                    mtrFrontRight.setPower(0);
                    mtrBackRight.setPower(0);
                    mtrBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    mtrFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    mtrFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    mtrBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    mtrBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    mtrFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    mtrFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    mtrBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    etLocal.reset();
                    iState++;
                }
            }
            else if (iState == 4) // move with the platform to the corner
            {
                mtrFrontLeft.setPower(1.0);
                mtrBackLeft.setPower(1.0);
                mtrFrontRight.setPower(1.0);
                mtrBackRight.setPower(1.0);
                if (mtrFrontLeft.getCurrentPosition() > MOVE_TO_CORNER || colorSensor.blue() > BLUE_LINE_DETECTION)
                {
                    mtrFrontLeft.setPower(0);
                    mtrBackLeft.setPower(0);
                    mtrFrontRight.setPower(0);
                    mtrBackRight.setPower(0);
                    mtrBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    mtrFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    mtrFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    mtrBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    mtrBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    mtrFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    mtrFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    mtrBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    // TODO: release the servos holding the platform
                    etLocal.reset();
                    iState++;
                }
            } // move with the platform to the corner
            else if (iState == 5) // turn and reverse for a few seconds (or until squared off at 90 degrees) to get off the blue line
            {
                mtrFrontLeft.setPower(-.20);
                mtrBackLeft.setPower(-0.20);
                mtrFrontRight.setPower(-0.6);
                mtrBackRight.setPower(-0.6);
                if (etLocal.seconds() > 1.5 || dDegrees < 90.0 )
                {
                    mtrFrontLeft.setPower(0);
                    mtrBackLeft.setPower(0);
                    mtrFrontRight.setPower(0);
                    mtrBackRight.setPower(0);
                    mtrBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    mtrFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    mtrFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    mtrBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    mtrBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    mtrFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    mtrFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    mtrBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    etLocal.reset();
                    iState++;
                }
            } // reversing to get off the blue line in the corner
            else if (iState == 6) // reverse to the center line
            {
                mtrFrontLeft.setPower(-.50);
                mtrBackLeft.setPower(-0.50);
                mtrFrontRight.setPower(-0.5);
                mtrBackRight.setPower(-0.5);
                if (colorSensor.blue() > BLUE_LINE_DETECTION)
                {
                    mtrFrontLeft.setPower(0);
                    mtrBackLeft.setPower(0);
                    mtrFrontRight.setPower(0);
                    mtrBackRight.setPower(0);
                    mtrBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    mtrFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    mtrFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    mtrBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    mtrBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    mtrFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    mtrFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    mtrBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    etLocal.reset();
                    iState++;
                }
            } // reversing to the center line

        }
        // Op mode is no longer active - stop the motors
        mtrBackLeft.setPower(0);
        mtrFrontLeft.setPower(0);
        mtrFrontRight.setPower(0);
        mtrBackRight.setPower(0);
    }
}
