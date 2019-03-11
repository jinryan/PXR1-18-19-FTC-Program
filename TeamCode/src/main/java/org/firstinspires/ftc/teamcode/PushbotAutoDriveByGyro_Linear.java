/* Copyright (c) 2017 FIRST. All rights reserved.
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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

/**
 * This file illustrates the concept of driving a path based on Gyro heading and encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that you have a Modern Robotics I2C gyro with the name "gyro"
 *   otherwise you would use: PushbotAutoDriveByEncoder;
 *
 *  This code requires that the drive Motors have been configured such that a positive
 *  power command moves them forward, and causes the encoders to count UP.
 *
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 *  In order to calibrate the Gyro correctly, the robot must remain stationary during calibration.
 *  This is performed when the INIT button is pressed on the Driver Station.
 *  This code assumes that the robot is stationary when the INIT button is pressed.
 *  If this is not the case, then the INIT should be performed again.
 *
 *  Note: in this example, all angles are referenced to the initial coordinate frame set during the
 *  the Gyro Calibration process, or whenever the program issues a resetZAxisIntegrator() call on the Gyro.
 *
 *  The angle of movement/rotation is assumed to be a standardized rotation around the robot Z axis,
 *  which means that a Positive rotation is Counter Clock Wise, looking down on the field.
 *  This is consistent with the FTC field coordinate conventions set out in the document:
 *  ftc_app\doc\tutorial\FTC_FieldCoordinateSystemDefinition.pdf
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Pushbot: Auto Drive By Gyro", group="Pushbot")
//@Disabled
public class PushbotAutoDriveByGyro_Linear extends LinearOpMode {

    /* Declare OpMode members. */
    PXR1Robot robot = new PXR1Robot();   // Use a Pushbot's hardware
    static final double     COUNTS_PER_MOTOR_REV    = 1680;
    private ElapsedTime runtime = new ElapsedTime();

    public double angletemp = 0;
    public double currentAngle = 0;
    public int counter = 0;

    double previousTime = 0.0;
    double currentTime = 0.0;
    double elapsedTime = 0.0;
    double currentError = 0.0;
    double previousError = 0.0;
    double proportional = 0.0;
    double derivative = 0.0;
    double integral = 0.0;
    double PID = 0.0;


    @Override
    public void runOpMode() {

        /*
         * Initialize the standard drive system variables.
         * The init() method of the hardware class does most of the work here
         */
        robot.init(hardwareMap);
        waitForStart();

//        encoderDriveByPos(15000,0.6,10);
//        turnByPID(30, 2);
        turnByPID(30,3);
        runtime.reset();
        while (runtime.seconds() < 10 && opModeIsActive()) {
            telemetry.addLine()
                    .addData("Right Encoder", robot.rightMotor.getCurrentPosition())
                    .addData("Left Encoder", robot.leftMotor.getCurrentPosition())
                    .addData("P", proportional)
                    .addData("I", integral)
                    .addData("D", derivative)
                    .addData("counter", counter)
                    .addData("Angle", currentAngle);
            updateAngle();
            telemetry.update();
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
            telemetry.addLine()
                    .addData("Right Encoder", robot.rightMotor.getCurrentPosition())
                    .addData("Left Encoder", robot.leftMotor.getCurrentPosition())
                    .addData("Angle", currentAngle);
            updateAngle();
            telemetry.update();
        }

        robot.leftMotor.setPower(0);
        robot.rightMotor.setPower(0);

        robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void driveByPID(int pos, double speed, double timeoutS) {
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.hMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        int newLeftTarget = robot.leftMotor.getCurrentPosition() + pos;
        int newRightTarget = robot.rightMotor.getCurrentPosition() + pos;
        robot.leftMotor.setTargetPosition(newLeftTarget);
        robot.rightMotor.setTargetPosition(newRightTarget);

        robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        angletemp = currentAngle;

        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < timeoutS && robot.leftMotor.isBusy() && robot.rightMotor.isBusy()) {
            currentError = currentAngle - angletemp;
            PID = calculatePID(currentError, previousError, 0.007, 0, 0, 0.2);
            previousError = currentError;
            robot.rightMotor.setPower(speed + PID);
            robot.leftMotor.setPower(speed + PID);
            updateAngle();
            telemetry.addData("angle:", currentAngle);
            telemetry.addData("target:", angletemp);
            telemetry.addData("power:", PID);
            telemetry.update();
        }
        currentError = 0;
        previousError = 0;
        robot.rightMotor.setPower(0);
        robot.leftMotor.setPower(0);
    }

    public void turnByPID(double targetAngle, double timeoutS) {
        runtime.reset();
        while (runtime.seconds() < timeoutS) {
            counter++;
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
        proportional = 0;
        derivative = 0;
        integral = 0;
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


    /**
     *  Method to drive on a fixed compass bearing (angle), based on encoder counts.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the desired position
     *  2) Driver stops the opmode running.
     *
     * @param speed      Target speed for forward motion.  Should allow for _/- variance for adjusting heading
     * @param distance   Distance (in inches) to move from current position.  Negative distance means move backwards.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     */

}