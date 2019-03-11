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
    private double clawOffset = 0.45;
    private double gatePosition = 0;
    private double CLAW_SPEED = 0.05;

    Button leftBumper = new Button();
    Button rightBumper = new Button();
    Button aButton = new Button();
    Button yButton = new Button();
    Button xButton = new Button();
    Button bButton = new Button();
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

        double frontPower = -gamepad1.right_stick_y;
        double sidePower = -gamepad1.right_stick_x;
        double hPower = -gamepad1.left_stick_x * microAdjustment * reverse;

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
        double x = Range.scale(robot.ods.getRawLightDetected(),0, 2.744, 1, 0);
        double y = Range.scale(robot.ods2.getRawLightDetected(), 0, 0.71, 1, 0);
        robot.hooker.setPower(gamepad2.right_trigger * x - gamepad2.left_trigger * y);
    }

    public void armDrive() {
        robot.arm.setPower(gamepad1.right_trigger - gamepad1.left_trigger);

    }

    public void spoolDrive() {
        if (gamepad1.right_bumper) {
            robot.spool.setPower(1);
        } else if (gamepad1.left_bumper) {
            robot.spool.setPower(-1);
        } else {
            robot.spool.setPower(gamepad1.right_trigger * 0.6 - gamepad1.left_trigger * 0.6);
        }
    }

    public void servoGate() {
        if (gamepad2.right_bumper)
            clawOffset += CLAW_SPEED;
        else if (gamepad2.left_bumper)
            clawOffset -= CLAW_SPEED;

        // Move both servos to new position.  Assume servos are mirror image of each other.
        clawOffset = Range.clip(clawOffset, -0.5, 0.5);
        robot.markerGate.setPosition(0.5 + clawOffset);
    }

    public void mineralGate() {
        if (gamepad1.dpad_right)
            gatePosition += CLAW_SPEED;
        else if (gamepad1.dpad_left)
            gatePosition -= CLAW_SPEED;

        // Move both servos to new position.  Assume servos are mirror image of each other.
        gatePosition = Range.clip(gatePosition, 0, 1);
        robot.mineralGate.setPosition(gatePosition);
    }

    public void sweeperDrive() {
        robot.mineralSweeper.setPower(gamepad1.left_stick_y);
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
        armDrive();
        spoolDrive();
        servoGate();
        sweeperDrive();
        mineralGate();
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
        double x = robot.imu.getPosition().x;
        double y = robot.imu.getPosition().y;
        double z = robot.imu.getPosition().z;
        telemetry.addData("Angle: ", currentAngle);
        telemetry.addLine()
            .addData("Position x", x)
            .addData("Position y", y)
            .addData("Position z", z);
        telemetry.addData("Left motor encoder:", robot.leftMotor.getCurrentPosition());
        telemetry.addData("Right motor encoder:", robot.rightMotor.getCurrentPosition());
        telemetry.addData("H motor encoder:", robot.hMotor.getCurrentPosition());
        telemetry.addData("Servo position", robot.markerGate.getPosition());
        telemetry.addData("MineralGate position", robot.mineralGate.getPosition());
        telemetry.addData("Sweeper power", robot.mineralSweeper.getPower());
        telemetry.addData("ODS raw:", robot.ods.getRawLightDetected());
        telemetry.addData("ODS2 raw:", robot.ods2.getRawLightDetected());
    }
}
