package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp
@Disabled
public class ArcadeDrive extends LinearOpMode {
    private DcMotor right;
    private DcMotor left;
    private float leftPower;
    private float rightPower;
    private float xValue;
    private float yValue;
    double drive;
    double turn;
    double max;

    @Override
    public void runOpMode() {
        left = hardwareMap.get(DcMotor.class, "left_drive");
        right = hardwareMap.get(DcMotor.class, "right_drive");
        right.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            yValue = gamepad1.left_stick_y;
            xValue = gamepad1.left_stick_x;


            leftPower = yValue - xValue;
            rightPower = yValue + xValue;

            left.setPower(Range.clip(leftPower, -1.0, 1.0));
            right.setPower(Range.clip(rightPower, -1.0, 1.0));

            telemetry.addData("stick", "  y=" + yValue + "  x=" + xValue);
            telemetry.addData("power", "  left=" + leftPower + "  right=" + rightPower);
            telemetry.addData("Left Motor Power", left.getPower());
            telemetry.addData("Right Motor Power", right.getPower());
            telemetry.addData("Status", "Running");
            telemetry.update();

            idle();
        }
    }
}