package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.*;

/**
 * Example OpMode. Demonstrates use of gyro, color sensor, encoders, and telemetry.
 *
 */
@SuppressWarnings({"unused", "WeakerAccess"})
@Disabled
@TeleOp(name = "arm bot demo", group = "ArmBot")
public class ArmBotDemo extends LinearOpMode {

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
        DcMotor m1 = hardwareMap.dcMotor.get("back_left_motor");
        DcMotor m2 = hardwareMap.dcMotor.get("front_left_motor");
        DcMotor m3 = hardwareMap.dcMotor.get("front_right_motor");
        DcMotor m4 = hardwareMap.dcMotor.get("back_right_motor");
        m1.setDirection(DcMotor.Direction.REVERSE);
        m2.setDirection(DcMotor.Direction.REVERSE);
        m1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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
        telemetry.addData("Encoders"," %d %d %d %d", m1.getCurrentPosition(), m2.getCurrentPosition(),
                m3.getCurrentPosition(), m4.getCurrentPosition());

/*
        telemetry.addData("Encoders"," %d %d %d %d", m1.getCurrentPosition(), m2.getCurrentPosition(),
                m3.getCurrentPosition(), m4.getCurrentPosition());

 */
        telemetry.update();

        gamepad1.setJoystickDeadzone(0.05f);

        waitForStart();
        boolean bButtonWasPressed = false;

        while (opModeIsActive()){
            double p1;
            double p2;
            double p3;
            double p4;

            if (bButtonWasPressed || gamepad1.x || gamepad1.y || gamepad1.a || gamepad1.b)
            {
                //             Buttons are arranged:
                //
                //                  Y
                //               X     B
                //                  A
                //
                // Buttons were pressed - use them and ignore the joystick
                bButtonWasPressed = gamepad1.x || gamepad1.y || gamepad1.a || gamepad1.b;

                if (gamepad1.y) // go forwards
                {
                    p1 = 1f;
                    p2 = 1f;
                    p3 = 1f;
                    p4 = 1f;
                }
                else if (gamepad1.x) // strafe left
                {
                    p1 = 1f;
                    p2 = -1f;
                    p3 = 1f;
                    p4 = -1f;
                }
                else if (gamepad1.a) // go backwards
                {
                    p1 = -1f;
                    p2 = -1f;
                    p3 = -1f;
                    p4 = -1f;
                }
                else if (gamepad1.b) // strafe right
                {
                    p1 = -1f;
                    p2 = 1f;
                    p3 = -1f;
                    p4 = 01f;
                }
                else // nothing pressed - stop
                {
                    p1 = 0f;
                    p2 = 0f;
                    p3 = 0f;
                    p4 = 0f;
                }
            }
            else
            {
                double px = gamepad1.left_stick_x;
                double py = -gamepad1.left_stick_y;
                double pa = gamepad1.left_trigger - gamepad1.right_trigger;
                if (Math.abs(pa) < 0.05) pa = 0;
                p1 = -px + py - pa;
                p2 = px + py + -pa;
                p3 = -px + py + pa;
                p4 = px + py + pa;
                double max = Math.max(1.0, Math.abs(p1));
                max = Math.max(max, Math.abs(p2));
                max = Math.max(max, Math.abs(p3));
                max = Math.max(max, Math.abs(p4));
                p1 /= max;
                p2 /= max;
                p3 /= max;
                p4 /= max;
            }

            m1.setPower(p1);
            m2.setPower(p2);
            m3.setPower(p3);
            m4.setPower(p4);
            arm.setPower(-gamepad1.right_stick_y);
            if (gamepad1.x) handServo.setPosition(1);
            else if (gamepad1.b) handServo.setPosition(0);
            telemetry.addData("Color","R %d  G %d  B %d", colorSensor.red(), colorSensor.green(), colorSensor.blue());
            //telemetry.addData("Heading"," %.1f", gyro.getHeading());
            Orientation orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
            telemetry.addData("Heading", " %.1f", orientation.firstAngle * 180.0 / Math.PI);

            telemetry.addData("Front Distance", getDistance(frontDistance));
            telemetry.addData("Left Distance", getDistance(leftDistance));
            telemetry.addData("Right Distance", getDistance(rightDistance));
            telemetry.addData("Back Distance", getDistance(backDistance));
            telemetry.addData("Encoders"," %d %d %d %d", m1.getCurrentPosition(), m2.getCurrentPosition(),
                    m3.getCurrentPosition(), m4.getCurrentPosition());
            telemetry.update();
        }
        // Op mode is no longer active - stop the motors
        m1.setPower(0);
        m2.setPower(0);
        m3.setPower(0);
        m4.setPower(0);
    }
}
