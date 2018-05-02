
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import java.util.Date;
import java.util.concurrent.TimeUnit;

@TeleOp
public class ConveyerTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        //servoTest = hardwareMap.get(, "left");
        DcMotor leftDrive = hardwareMap.dcMotor.get("left_drive");
        DcMotor rightDrive = hardwareMap.dcMotor.get("right_drive");
        leftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        double leftPower = 0;
        double rightPower = 0;

        DcMotor belt1 = hardwareMap.dcMotor.get("belt1");
        DcMotor belt2 = hardwareMap.dcMotor.get("belt2");
        belt1.setDirection(DcMotorSimple.Direction.FORWARD);
        belt2.setDirection(DcMotorSimple.Direction.REVERSE);
        belt1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        belt2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        double beltPower = 0;

        DcMotor elevator = hardwareMap.dcMotor.get("elevator");
        DcMotor dump = hardwareMap.dcMotor.get("dump");
        elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        dump.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        int elevatorPosition = 0;
        int elevatorStatus = 0;

        DcMotor relicMotor = hardwareMap.dcMotor.get("relicMotor");
        relicMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        boolean relicArmPosition = false;
        double relicArmPower = 0;
        Servo relicExtend = hardwareMap.servo.get("relicExtend");
        Servo relicHand = hardwareMap.servo.get("relicHand");
        double          clawOffset      = 0;
        double          clawOffset2     = 0;
        final double    CLAW_SPEED      = 0.08 ;

        Date stamp = new Date();


        boolean position1 = false;
        boolean position2 = false;
        waitForStart();

        while (opModeIsActive()) {
            Date loopDate = new Date();
            long diff = loopDate.getTime() - stamp.getTime();

            if (diff >= 50) {
                stamp = new Date();
                if (gamepad2.dpad_up) {
                    clawOffset += CLAW_SPEED;
                } else if (gamepad2.dpad_down) {
                    clawOffset -= CLAW_SPEED;
                }

                if (gamepad2.dpad_left) {
                    clawOffset2 += CLAW_SPEED;
                } else if (gamepad2.dpad_right) {
                    clawOffset2 -= CLAW_SPEED;
                }
            }

            clawOffset = Range.clip(clawOffset, -0.5, 0.5);
            clawOffset2 = Range.clip(clawOffset2, -0.5, 0.5);
            relicExtend.setPosition(0.5 + clawOffset2);
            relicHand.setPosition(0.5 + clawOffset);

            relicArmPower = gamepad2.right_stick_y;
            relicMotor.setPower(Range.clip(relicArmPower, -0.5, 0.5));


            leftPower = Range.clip(gamepad1.left_stick_y, -0.5, 0.5);
            rightPower = Range.clip(gamepad1.right_stick_y, -0.5, 0.5);

            leftDrive.setPower(leftPower);
            rightDrive.setPower(rightPower);

            beltPower = Range.clip(gamepad2.left_stick_y, -0.5, 0.5);

            belt1.setPower(beltPower);
            belt2.setPower(beltPower);

            if(gamepad2.a) {
                if(elevatorStatus == 0) {
                    elevatorPosition = 2000;
                    elevator.setTargetPosition(elevatorPosition);
                    elevator.setPower(0.5);
                    while(elevator.isBusy()) {
                        wait();
                    }
                    elevator.setPower(0.0);
                    elevatorStatus = 1;
                }
                if(elevatorStatus == 1) {
                    elevatorPosition = elevatorPosition + 2000;
                    elevator.setTargetPosition(elevatorPosition);
                    elevator.setPower(0.5);
                    while(elevator.isBusy()) {
                        wait();
                    }
                    elevator.setPower(0.0);
                    elevatorStatus = 2;
                }
                }

            if(gamepad2.b) {
                if (elevatorStatus == 1) {
                    elevatorPosition = 2000;
                    elevator.setTargetPosition(elevatorPosition);
                    elevator.setPower(-0.5);
                    while (elevator.isBusy()) {
                        wait();
                    }
                    elevator.setPower(0.0);
                    elevatorStatus = 0;
                }
                if (elevatorStatus == 2) {
                    elevatorPosition = 4000;
                    elevator.setTargetPosition(elevatorPosition);
                    elevator.setPower(-0.5);
                    while (elevator.isBusy()) {
                        wait();
                    }
                    elevator.setPower(0.0);
                    elevatorStatus = 0;
                }
            }





            telemetry.addData("Glyph Lift Status:", elevatorStatus);
            telemetry.addData("Glyph Lift Position:", elevatorPosition);
            telemetry.addData("Belt Power:", beltPower);
            telemetry.addData("Left Drive:", leftDrive);
            telemetry.addData("Right Drive:", rightDrive);
            telemetry.update();
        }

    }
}





















