package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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
 * Builds off the first arm bot auto with different paths and sensor usage
 *
 */
@SuppressWarnings({"unused", "WeakerAccess"})
@Autonomous(name = "arm bot auto 2", group = "ArmBot")
public class ArmBotAuto2 extends LinearOpMode {

    private int BLUE_LINE_DETECTION = 200;
    private int REVERSE_90_ENCODER = 5000;
    private int REVERSE_AFTER_PLATFORM = 1500;
    private int MOVE_TO_CORNER = 8000;
    private double DISTANCE_TO_WALL = 15.0;
    private ElapsedTime etLocal = new ElapsedTime();
    private ElapsedTime etOpMode = new ElapsedTime();

    private String getDistance(DistanceSensor sensor)
    {
        double dRawDistance = sensor.getDistance(DistanceUnit.CM);
        return dRawDistance < 815 ?  String.format("%f", dRawDistance) : "Undefined";
    }

    /**
     * This is a drop-in replacement for telemetry.addData (sCaption, sStringToLog). However, it
     * also logs the data to stdout with a following tab. This allows the output to be logged as
     * tab-separated files. You will need to manually add a linefeed after each set of data.
     * @param sCaption The caption to send to telemetry. Will not be output to the log file.
     * @param oToLog The Object/data to log. Will be converted to a string.
     */
    private void logTwice(String sCaption, Object oToLog)
    {
        telemetry.addData(sCaption, oToLog);
        System.out.print( oToLog.toString() + "\t");
    }

    public void runOpMode(){
        telemetry.addData("Hardware mapping","begin");
        telemetry.update();
        DcMotor mtrBackLeft = hardwareMap.dcMotor.get("back_left_motor");
        DcMotor mtrFrontLeft = hardwareMap.dcMotor.get("front_left_motor");
        DcMotor mtrFrontRight = hardwareMap.dcMotor.get("front_right_motor");
        DcMotor mtrBackRight = hardwareMap.dcMotor.get("back_right_motor");
        DcMotor arm = hardwareMap.dcMotor.get("arm_motor");
        //GyroSensor gyro = hardwareMap.gyroSensor.get("gyro_sensor");
        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        Servo handServo = hardwareMap.servo.get("hand_servo");
        ColorSensor colorSensor = hardwareMap.colorSensor.get("color_sensor");
        DistanceSensor frontDistance = hardwareMap.get(DistanceSensor.class, "front_distance");
        DistanceSensor leftDistance = hardwareMap.get(DistanceSensor.class, "left_distance");
        DistanceSensor rightDistance = hardwareMap.get(DistanceSensor.class, "right_distance");
        DistanceSensor backDistance = hardwareMap.get(DistanceSensor.class, "back_distance");

        logTwice("Hardware mapping","map end, config begin");
        telemetry.update();

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

        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        logTwice("Hardware mapping","config end, imu init begin");
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
        logTwice("Hardware mapping","imu init end");
        telemetry.update();

        logTwice("Press Start Button When Ready","");
        logTwice("Left Distance", getDistance(leftDistance));
        logTwice("Encoders", String.format(" %d %d %d %d", mtrBackLeft.getCurrentPosition(), mtrFrontLeft.getCurrentPosition(),
                mtrFrontRight.getCurrentPosition(), mtrBackRight.getCurrentPosition()));
        telemetry.update();

        waitForStart();
        etOpMode.reset();
        etLocal.reset();

        int iState = 0;
        System.out.println("Red\tGreen\tBlue\tHeading\tState\tLeft_Distance\tBack_Left\tFront_Left\tFront_Right\tBack_Right\tState_Time\tOpmode_Time");

        while (opModeIsActive()){
            logTwice("Color", String.format("%d\t%d\t%d", colorSensor.red(), colorSensor.green(), colorSensor.blue()));
            //telemetry.addData("Heading"," %.1f", gyro.getHeading());
            Orientation orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
            double dDegrees = orientation.firstAngle * 180.0 / Math.PI;
            logTwice("Heading", String.format(" %.1f", dDegrees));
            logTwice("State", iState);
            logTwice("Left Distance", getDistance(leftDistance));
            logTwice("Encoders",String.format(" %d\t%d\t%d\t%d", mtrBackLeft.getCurrentPosition(), mtrFrontLeft.getCurrentPosition(),
                    mtrFrontRight.getCurrentPosition(), mtrBackRight.getCurrentPosition()));
            logTwice("Local Runtime",  etLocal.seconds());
            logTwice("Op Mode Runtime",  etOpMode.seconds());
            System.out.println("");

            telemetry.update();

            if (iState == 0) // Strafe left until the blue line is detected or distance to wall is reached, then stop and advance the state
            {
                if (colorSensor.blue() < BLUE_LINE_DETECTION && leftDistance.getDistance(DistanceUnit.CM) > DISTANCE_TO_WALL)
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
            } // strafe left until the blue line is detected or distance to wall is reached, then stop and advance the state
            else if (iState == 1) // blue line was detected; advance forward at least 1.5 seconds until the blue line is no longer detected
            {
                if (colorSensor.blue() > BLUE_LINE_DETECTION || etLocal.seconds() < 0.5)
                {
                    mtrFrontLeft.setPower(0.75);
                    mtrBackLeft.setPower(0.75);
                    mtrFrontRight.setPower(0.75);
                    mtrBackRight.setPower(0.75);
                }
                else // blue line no longer detected; advance the state, but no need to stop
                {
                    etLocal.reset();
                    iState++;
                }
            } // advancing forward until off the blue line
            else if (iState == 2) // moving forward to acquire the platform
            {
                if (colorSensor.blue() < BLUE_LINE_DETECTION)
                {
                    mtrFrontLeft.setPower(0.75);
                    mtrBackLeft.setPower(0.75);
                    mtrFrontRight.setPower(0.75);
                    mtrBackRight.setPower(0.75);
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
            else if (iState == 3) // grab the platform and reverse a bit
            {
                // TODO: lower servo
                mtrFrontLeft.setPower(-0.50);
                mtrBackLeft.setPower(-0.50);
                mtrFrontRight.setPower(-0.50);
                mtrBackRight.setPower(-0.50);
                // Check an encoder to make sure you don't go too far
                if (mtrFrontLeft.getCurrentPosition() < -REVERSE_AFTER_PLATFORM || etLocal.seconds() > 5.0)
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
            } // grab the platform and reverse a bit
            else if (iState == 4) // after reversing a bit, turn
            {
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
            } // after reversing a bit, turn
            else if (iState == 5) // move with the platform to the corner
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
            else if (iState == 6) // turn and reverse for a few seconds (or until squared off at 90 degrees) to get off the blue line
            {
                mtrFrontLeft.setPower(-.20);
                mtrBackLeft.setPower(-0.20);
                mtrFrontRight.setPower(-0.4);
                mtrBackRight.setPower(-0.4);
                if ( dDegrees < 90.0 && etLocal.seconds() > 0.5 )
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
            else if (iState == 7) // reverse to the center line
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
