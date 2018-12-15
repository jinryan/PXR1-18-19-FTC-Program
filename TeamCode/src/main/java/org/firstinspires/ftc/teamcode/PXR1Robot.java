package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class PXR1Robot {
    /* Public OpMode members. */
    public ElapsedTime runTime = new ElapsedTime();

    // ======================== Hardware =======================


    // Drive base
    public DcMotor leftMotor = null;
    public DcMotor rightMotor = null;
    public DcMotor hMotor = null;
    public DcMotor hooker = null;
    public BNO055IMU imu = null;
    public TouchSensor touch = null;


    // ======================= Parameters ==================

    public double rightSpeedToLeftRatio = 0.9;

    // Claw parameters


    /* local OpMode members. */
    HardwareMap hardwareMap           =  null;


    /* Constructor */

    public PXR1Robot(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hardwareMap = ahwMap;

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Define and Initialize Motors
        leftMotor = hardwareMap.get(DcMotor.class, "left");
        rightMotor = hardwareMap.get(DcMotor.class, "right");
        hMotor = hardwareMap.get(DcMotor.class, "h");
        hooker = hardwareMap.get(DcMotor.class, "hooker");
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        touch = hardwareMap.get(TouchSensor.class, "touch");
        imu.initialize(parameters);

        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hooker.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftMotor.setDirection(DcMotor.Direction.REVERSE);
//        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    void fit18() {

    }

    public void runWithDegrees(int leftDegrees, int rightDegrees, double leftPower, double rightPower) {

        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftMotor.setTargetPosition(leftDegrees);
        rightMotor.setTargetPosition(rightDegrees);

        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);



        drive(leftPower, rightPower, 0);
        while (leftMotor.isBusy() && rightMotor.isBusy()) {}

        stopDrive();

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void drive(double speedLeft, double speedRight, double speedH) {
        leftMotor.setPower(speedLeft * rightSpeedToLeftRatio);
        rightMotor.setPower(speedRight);
        hMotor.setPower(speedH);
    }


    public void stopDrive() {
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }

    public void runWithTime(double leftPower, double rightPower, double time, boolean stopAfter) {
        drive(leftPower, rightPower, 0);
        runTime.reset();
        while (runTime.seconds() < time) {}
        if (stopAfter)
            stopDrive();
    }

}

