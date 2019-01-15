package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@TeleOp(name="Single Driver Opmode", group="TeleOp")

public class DriverOneControlOpmode extends OpMode
{
    private PXR1Robot robot = null;
    ElapsedTime runTime = null;

    private double microAdjustment = 1;
    private int reverse = 1;

    Button leftBumper = new Button();
    Button rightBumper = new Button();
    Button aButton = new Button();
    Button yButton = new Button();
    Button upButton = new Button();
    Button downButton = new Button();




    // Microadjustment
    public void updateMicroadjust() {
        if (aButton.rebound(gamepad1.a)) {
            if (microAdjustment == 1.0) {
                microAdjustment = 0.25;
            } else {
                microAdjustment = 1.0;
            }
        }
    }

    // Reverse
    public void updateReverse() {
        if (yButton.rebound(gamepad1.y)) {
            reverse *= -1;
        }
    }

    private void manualDrive() {

        double frontPower = -gamepad1.left_stick_y;
        double sidePower = -gamepad2.left_stick_x;
        double hPower = -gamepad1.left_stick_x * microAdjustment;

        double overridingSidePower = -gamepad1.right_stick_x;

        if (Math.abs(overridingSidePower) > 0.2)
            sidePower = overridingSidePower;

        double leftPower = frontPower * microAdjustment * reverse - sidePower * (1+(1 - microAdjustment) * -0.6);
        double rightPower = frontPower * microAdjustment * reverse + sidePower * (1+(1 - microAdjustment) * -0.6);

        // In case it overflows
        leftPower    = Range.clip(leftPower, -1.0, 1.0) ;
        rightPower   = Range.clip(rightPower, -1.0, 1.0) ;
        telemetry.addData("Left Power", leftPower);
        telemetry.addData("Right Power", rightPower);
        // Setting power
        robot.drive(leftPower, rightPower, hPower);
    }

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        robot = new PXR1Robot();
        runTime = new ElapsedTime();
        robot.init(hardwareMap);
        leftBumper.init();
        rightBumper.init();
        aButton.init();
        yButton.init();
        upButton.init();
        downButton.init();
        telemetry.addData("Status", "Initialized");
        robot.fit18();
    }

    public void hookerDrive() {
        double x = Range.scale(robot.ods.getRawLightDetected(),0, 1.8, 1, 0);
        double y = Range.scale(robot.ods2.getRawLightDetected(), 0, 0.71, 1, 0);
        robot.hooker.setPower(gamepad2.right_trigger * x - gamepad2.left_trigger * y);
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        // If you are using Motorola E4 phones,
        // you should send telemetry data while waiting for start.
        telemetry.addData("status", "loop test... waiting for start");
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {

    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        updateMicroadjust();
        manualDrive();
        updateReverse();
        hookerDrive();
        updateMotorEncoderValues();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }


    private void updateMotorEncoderValues() {
        double currentAngle = robot.imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        telemetry.addData("Angle: ", currentAngle);
        telemetry.addData("Left motor encoder:", robot.leftMotor.getCurrentPosition());
        telemetry.addData("Right motor encoder:", robot.rightMotor.getCurrentPosition());
        telemetry.addData("H motor encoder:", robot.hMotor.getCurrentPosition());
        telemetry.addData("ODS:", robot.ods.getLightDetected());
        telemetry.addData("ODS:", robot.ods.getRawLightDetected());
        telemetry.addData("ODS2:", robot.ods2.getLightDetected());
        telemetry.addData("ODS2:", robot.ods2.getRawLightDetected());
    }
}
