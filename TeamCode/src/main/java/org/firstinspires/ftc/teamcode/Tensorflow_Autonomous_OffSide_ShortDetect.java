/* Copyright (c) 2018 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

/**
 * This 2018-2019 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine the position of the gold and silver minerals.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
@Autonomous(name = "Autonomous: TensorFlow Offside short", group = "Autonomous")
//@Disabled
public class Tensorflow_Autonomous_OffSide_ShortDetect extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    private ElapsedTime runtime = new ElapsedTime();

    private int position = -1;

    private double currentAngle = 0;
    private double x = 0;
    private double angletemp = 0;

    double previousTime = 0.0;
    double currentTime = 0.0;
    double elapsedTime = 0.0;
    double currentError = 0.0;
    double previousError = 0.0;
    double proportional = 0.0;
    double derivative = 0.0;
    double integral = 0.0;
    double PID = 0.0;

    static final double     COUNTS_PER_MOTOR_REV    = 1680;
    static final double     DRIVE_GEAR_REDUCTION    = 0.667;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    PXR1Robot robot = new PXR1Robot();

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY = "AdMfwt3/////AAAAGb1teOGtfEIMp2su4rECQC8XtaXGZ31RT/lGzygte/bMFzL08u3r8XwZIq+LZyydLQ2c/zPv9keO43DluoznWwtCL7F8VT23sSONaoyNjho5L7W/Xc4e1Ee5WKYq2v3X9Fjrhfos5kbo1g9bn4X27IUnjEEgviP0AmanRhc8rC1OnCS2HczyvE0KLjhSBBN5GLyu5BqgwQ+RhjKJZY79E3vvUxj35w5wjKGCVoPpCNASH73AenBLtIA3ZxvXWJAvFA46D0Ntu0N/HGKyjQpPPVfd5vXRCiY32VXrPNCTy2OLqIrcjCA947tneXmNCp9M3Ylc/7MUPw1S5WH4mOXLQBnhco3aIiPTW5dkqItRDP6q";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the Tensor Flow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        robot.init(hardwareMap);

        currentAngle = robot.imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();
//      waitForStart();
        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("status", "waiting for start command...");
            telemetry.update();
        }

        //lower from lander
        if (opModeIsActive()) {
            runtime.reset();
            while (opModeIsActive() && runtime.seconds() < 4.5) {
                x = Range.scale(robot.ods.getRawLightDetected(),0, 2.744, 1, 0);
                robot.hooker.setPower(1 * x);
            }
            robot.hooker.setPower(0);
            encoderSlideByPos(600, 1, 3);

            /** Activate Tensor Flow Object Detection. */
            if (tfod != null) {
                tfod.activate();
            }

            //detect gold mineral
            while (opModeIsActive() && runtime.seconds() < 4 && position == -1) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        if (updatedRecognitions.size() == 2) {
                            double pos1 = 0;
                            double pos2 = 0;
                            pos1 = updatedRecognitions.get(0).getLeft();
                            pos2 = updatedRecognitions.get(1).getLeft();
                            if (updatedRecognitions.get(0).getLabel().equals(LABEL_GOLD_MINERAL) && pos1 < pos2 || updatedRecognitions.get(1).getLabel().equals(LABEL_GOLD_MINERAL) && pos2 < pos1) {
                                position = 0;
                            } else if (updatedRecognitions.get(0).getLabel().equals(LABEL_GOLD_MINERAL) && pos1 > pos2 || updatedRecognitions.get(1).getLabel().equals(LABEL_GOLD_MINERAL) && pos2 > pos1) {
                                position = 1;
                            } else {
                                position = 2;
                            }
                            telemetry.addData("position", position);
                            telemetry.update();
                        } else if (updatedRecognitions.size() == 3) {
                            int goldMineralX = -1;
                            int silverMineral1X = -1;
                            int silverMineral2X = -1;
                            for (Recognition recognition : updatedRecognitions) {
                                if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                    goldMineralX = (int) recognition.getLeft();
                                } else if (silverMineral1X == -1) {
                                    silverMineral1X = (int) recognition.getLeft();
                                } else {
                                    silverMineral2X = (int) recognition.getLeft();
                                }
                            }
                            if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
                                if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
                                    telemetry.addData("Gold Mineral Position", "Left");
                                    position = 0;
                                } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                                    telemetry.addData("Gold Mineral Position", "Right");
                                    position = 2;
                                } else {
                                    telemetry.addData("Gold Mineral Position", "Center");
                                    position = 1;
                                }
                            }
                            telemetry.update();
                        }
                        telemetry.update();
                    }
                }
            }

            //move in front of the gold mineral
            encoderDriveByPos(500, 0.8, 1.5);

            if (position == -1) {
                position = 1;
            }

            if (position == 0) {//left
                turnByPID(28, 1.5);
                encoderDriveByPos(2800,1,3);
                encoderDriveByPos(-1500, 1, 3);
                turnByPID(90,3);
                encoderDriveByPos(3500,0.8,3);
            } else if (position == 1) {//middle
                encoderDriveByPos(2800,1,3);
                encoderDriveByPos(-1300,1,3);
                turnByPID(90,3);
                encoderDriveByPos(4000,0.8,3);
            } else if (position == 2) {//right
                turnByPID(-37,1.5);
                encoderDriveByPos(2800,1,3);
                encoderDriveByPos(-1500,1,3);
                turnByPID(90,3);
                encoderDriveByPos(4500,0.8,3);
            }
            turnByPID(135,1.5);
            encoderSlideByPos(-2400, 1, 3);
            encoderSlideByPos(200,1,1);
            encoderDriveByPos( 4600,1,4);
            robot.markerGate.setPosition(0.5);
            runtime.reset();
            while (runtime.seconds() < 1.5) {}
            encoderDriveByPos(-15000,1,4);
        }

        if (tfod != null) {
            tfod.shutdown();
        }
    }

    public void encoderDriveByPos(int pos, double speed, double timeoutS) {
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.hMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        int newLeftTarget = robot.leftMotor.getCurrentPosition() + pos;
        int newRightTarget = robot.rightMotor.getCurrentPosition() + pos;
        robot.leftMotor.setTargetPosition(newLeftTarget);
        robot.rightMotor.setTargetPosition(newRightTarget);

        robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.leftMotor.setPower(Math.abs(speed));
        robot.rightMotor.setPower(Math.abs(speed));
        runtime.reset();
        while (opModeIsActive() &&
                (runtime.seconds() < timeoutS) &&
                (robot.leftMotor.isBusy() && robot.rightMotor.isBusy())) {
        }

        robot.leftMotor.setPower(0);
        robot.rightMotor.setPower(0);

        robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void encoderSlideByPos(int pos, double speed, double timeoutS) {
        robot.hMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        int newSlideTarget = robot.hMotor.getCurrentPosition() + pos;
        robot.hMotor.setTargetPosition(newSlideTarget);

        robot.hMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.hMotor.setPower(Math.abs(speed));

        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < timeoutS && robot.hMotor.isBusy()) {
        }

        robot.hMotor.setPower(0);

        robot.hMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void turnByPID(double targetAngle, double timeoutS) {
        runtime.reset();
        while (runtime.seconds() < timeoutS) {
            updateAngle();
            currentError = currentAngle - targetAngle;
            PID = calculatePID(currentError, previousError, robot.kp, robot.kd, robot.ki, 1);
            robot.rightMotor.setPower(PID);
            robot.leftMotor.setPower(-PID);
            previousError = currentError;
            telemetry.addData("angle:", currentAngle);
            telemetry.addData("target:", targetAngle);
            telemetry.addData("power:", PID);
            telemetry.update();
        }
        currentError = 0;
        previousError = 0;
        robot.rightMotor.setPower(0);
        robot.leftMotor.setPower(0);
    }

    public double calculatePID(double error, double previousError, double kp, double kd, double ki, double range)  {
        previousTime = currentTime;
        currentTime = runtime.milliseconds();
        elapsedTime = (currentTime - previousTime) / 1000;
        proportional = error * kp;
        integral = integral + (error * ki);
        derivative = kd * ((error - previousError) / elapsedTime);
        PID = proportional + integral + derivative;
        PID = -Range.clip(PID, -range, range);
        telemetry.addData("power:", PID);
        telemetry.update();
        return PID;
    }


    public void updateAngle() {
        currentAngle = robot.imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }

    public int[] resetArray(int[] a) {
        for (int i = 0; i < a.length; i++) {
            a[i] = -1;
        }
        return a;
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
        parameters.cameraDirection = CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
            "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.7;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }
}
