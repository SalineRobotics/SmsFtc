/*

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;/*package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.FLOAT;

@TeleOp
public class SlowDown extends LinearOpMode {
    private DcMotor right;
    private DcMotor left;
    private float leftPower;
    private float rightPower;
    private float xValue;
    private float yValue;
    private int leftPosition;
    private int rightPosition;



    @Override
    public void runOpMode() {
        left = hardwareMap.get(DcMotor.class, "left");
        right = hardwareMap.get(DcMotor.class, "right");
        right.setDirection(DcMotor.Direction.REVERSE);
        left.setZeroPowerBehavior(FLOAT);
        right.setZeroPowerBehavior(FLOAT);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if(gamepad1.a) {
                left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    leftPosition = left.getCurrentPosition();
                    rightPosition = right.getCurrentPosition();

                    left.setTargetPosition(left.getCurrentPosition() + 4000);
                    right.setTargetPosition(right.getCurrentPosition() + 4000);

                    left.setPower(0.5);
                    right.setPower(0.5);

                    while (left.isBusy() && right.isBusy()) {
                        idle();
                    }

                    left.setPower(0);
                    right.setPower(0);
            }

            if(gamepad2.b) {
                left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                leftPosition = left.getCurrentPosition();
                rightPosition = right.getCurrentPosition();

                left.setPower(0.5);
                right.setPower(0.5);
                telemetry.addData("Power", left.getPower());
                telemetry.update();


                while(left.getCurrentPosition() < leftPosition + 6000 && right.getCurrentPosition() < rightPosition + 6000) {
                    leftPosition = left.getCurrentPosition();
                    rightPosition = right.getCurrentPosition();

                    if (left.getCurrentPosition() < leftPosition + 6000)
                    {
                        left.setPower(0.5);
                        right.setPower(0.5);
                 1111111111111111111   } else if(left.getCurrentPosition() < leftPosition + 3000 ) {
                        left.setPower(0.3);
                        right.setPower(0.3);
                    }

                    idle();
                }

                while(left.getCurrentPosition() < leftPosition + 3000 && right.getCurrentPosition() < rightPosition + 3000) {
                    idle();
                }
                leftPosition = left.getCurrentPosition();
                rightPosition = right.getCurrentPosition();
                left.setPower(0.3);
                right.setPower(0.3);
                telemetry.addData("Power", left.getPower());
                telemetry.update();

                while(left.getCurrentPosition() < leftPosition + 2000 && right.getCurrentPosition() < rightPosition + 2000) {
                    idle();
                }

                leftPosition = left.getCurrentPosition();
                rightPosition = right.getCurrentPosition();

                left.setPower(0.1);
                right.setPower(0.1);
                telemetry.addData("Power", left.getPower());
                telemetry.update();
                while(left.getCurrentPosition() < leftPosition + 1000 && right.getCurrentPosition() < rightPosition + 1000) {
                    idle();
                }

                left.setPower(0);
                right.setPower(0);
                telemetry.addData("Power", left.getPower());
                telemetry.update();
            }




            telemetry.addData("Status", "Running");
            telemetry.update();

            idle();
        }
    }
}

*/