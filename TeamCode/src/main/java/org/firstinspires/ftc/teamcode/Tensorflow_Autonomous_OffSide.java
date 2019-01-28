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
@Autonomous(name = "Autonomous: TensorFlow Offside", group = "Autonomous")
//@Disabled
public class Tensorflow_Autonomous_OffSide extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    private ElapsedTime runtime = new ElapsedTime();

    private int position = -1;
    private double currentAngle = 0;
    private double x = 0;

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

        if (opModeIsActive()) {
            /** Activate Tensor Flow Object Detection. */
            if (tfod != null) {
                tfod.activate();
            }

            runtime.reset();
            while (opModeIsActive() && runtime.seconds() < 5.5) {
                x = Range.scale(robot.ods.getRawLightDetected(),0, 1.7, 1, 0);
                robot.hooker.setPower(1 * x);
            }
            robot.hooker.setPower(0);
            encoderSlideByPos(1000, 1, 3);

            while (opModeIsActive()) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        if (updatedRecognitions.size() == 3) {
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
                            break;
                        } else if (updatedRecognitions.size() == 2) {
                            boolean left = true;
                            boolean middle = true;
                            for (Recognition recognition : updatedRecognitions) {
                                telemetry.addData("Position", recognition.getLeft());
                                if (recognition.getLeft() < 400 && recognition.getLabel().equals(LABEL_SILVER_MINERAL)) {
                                    left = false;
                                } else if (recognition.getLeft() > 600 && recognition.getLabel().equals(LABEL_SILVER_MINERAL)) {
                                    middle = false;
                                }
                            }
                            if (!left && !middle) {
                                position = 2;
                            } else if (!left && middle) {
                                position = 1;
                            } else {
                                position = 0;
                            }
                            telemetry.update();
                            break;
                        }
                        telemetry.update();

                    }
                }
            }

            encoderDriveByPos(500, 0.6, 1.5);
                //move in front of the gold mineral
            if (position == 0) {
                turnByAngle(20, 0.4, 3);
                encoderDriveByPos(3500,0.8,3);
                encoderDriveByPos(-3500, 0.8, 3);
//                turnByAngle(70, 0.4, 3);
//                encoderDriveByPos(5000, 0.8, 5);
//                turnByAngle(45, 0.4, 4);
//                encoderDriveByPos(5000, 0.8, 5);
            } else if (position == 1) {
                encoderDriveByPos(3500,0.8,3);
                encoderDriveByPos(-3500, 0.8, 3);
//                turnByAngle(90, 0.4, 3);
//                encoderDriveByPos(5000, 0.8, 5);

            } else if (position == 2) {
                turnByAngle(-20, 0.4, 3);
                encoderDriveByPos(3500,0.8,3);
                encoderDriveByPos(-3500, 0.8, 3);
//                turnByAngle(110, 0.4, 4);
//                encoderDriveByPos(5000, 0.8, 5);
//                turnByAngle(45, 0.4, 4);
//                encoderDriveByPos(5000, 0.8, 5);

            }
        }

        if (tfod != null) {
            tfod.shutdown();
        }
    }

    public void encoderDriveByPos(int pos, double speed, double timeoutS) {
        int newLeftTarget = robot.leftMotor.getCurrentPosition() + pos;
        int newRightTarget = robot.rightMotor.getCurrentPosition() + pos;
        robot.leftMotor.setTargetPosition(newLeftTarget);
        robot.rightMotor.setTargetPosition(newRightTarget);

        robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.leftMotor.setPower(Math.abs(speed));
        robot.rightMotor.setPower(Math.abs(speed));

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
        int newSlideTarget = robot.hMotor.getCurrentPosition() + pos;
        robot.hMotor.setTargetPosition(newSlideTarget);

        robot.hMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.hMotor.setPower(Math.abs(speed));

        while (opModeIsActive() &&
                (runtime.seconds() < timeoutS) && robot.hMotor.isBusy()) {
        }

        robot.hMotor.setPower(0);

        robot.hMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void turnByAngle(double angle, double speed, double timeoutS) {
        runtime.reset();
        if (angle > 0) {
            robot.leftMotor.setPower(-speed);
            robot.rightMotor.setPower(speed);
            while (currentAngle < angle && opModeIsActive() && runtime.seconds() < timeoutS) {
                updateAngle();
                telemetry.addData("Angle: ", currentAngle);
                telemetry.update();
            }
            robot.leftMotor.setPower(0);
            robot.rightMotor.setPower(0);
        } else {
            robot.leftMotor.setPower(speed);
            robot.rightMotor.setPower(-speed);
            while (currentAngle > angle && opModeIsActive() && runtime.seconds() < timeoutS){
                updateAngle();
                telemetry.addData("Angle: ", currentAngle);
                telemetry.update();
            }
            robot.leftMotor.setPower(0);
            robot.rightMotor.setPower(0);
        }
    }

    public void updateAngle() {
        currentAngle = robot.imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
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
        tfodParameters.minimumConfidence = 0.8;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }
}
