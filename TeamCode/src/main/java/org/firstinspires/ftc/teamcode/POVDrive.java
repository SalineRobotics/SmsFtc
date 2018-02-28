/*package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@TeleOp
public class POVDrive extends LinearOpMode {
    private DcMotor rightMotor;
    private DcMotor leftMotor;
    private float left;
    private float right;
    private float leftPower;
    private float rightPower;
    private float xValue;
    private float yValue;
    double drive;
    double turn;
    double max;

    @Override
    public void runOpMode() {
        leftMotor = hardwareMap.get(DcMotor.class, "left_drive");
        rightMotor = hardwareMap.get(DcMotor.class, "right_drive");
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        double left;
        double right;
        double drive;
        double turn;
        double max;
        double speed = 0.5;

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            drive = -gamepad1.left_stick_y;
            turn  =  -gamepad1.right_stick_x;

            if(gamepad1.x) {
                speed = 1.0;
            }

            if(gamepad1.b) {
                speed = 0.5;
        }

            if(gamepad1.y) {
                speed = 0.25;
            }
            // Combine drive and turn for blended motion.
            left  = drive + turn;
            right = drive - turn;

            // Normalize the values so neither exceed +/- 1.0
            max = Math.max(Math.abs(left), Math.abs(right));
            if (max > 1.0)
            {
                left /= max;
                right /= max;
            }

            leftMotor.setPower(Range.clip(left,-speed, speed));
            rightMotor.setPower(Range.clip(right,-speed, speed));

            telemetry.addData("Left Motor Power", leftMotor.getPower());
            telemetry.addData("Right Motor Power", rightMotor.getPower());
            telemetry.addData("Status", "Running");
            telemetry.update();

            idle();
        }
    }
}*/