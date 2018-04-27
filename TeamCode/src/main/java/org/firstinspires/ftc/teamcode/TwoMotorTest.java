
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

@TeleOp
public class TwoMotorTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        //servoTest = hardwareMap.get(, "left");
        DcMotor motor1 = hardwareMap.dcMotor.get("left_drive");
        DcMotor motor2 = hardwareMap.dcMotor.get("right_drive");
        motor1.setDirection(DcMotorSimple.Direction.FORWARD);
        motor2.setDirection(DcMotorSimple.Direction.REVERSE);
        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        double leftPower = 0;
        double rightPower = 0;
        int speed = 3;
        waitForStart();

        while (opModeIsActive()) {

            if(gamepad1.x) {
                speed = 1;
            }

            if(gamepad1.b) {
                speed = 2;
            }

            if(gamepad1.a) {
                speed = 3;
            }

            if(gamepad1.y) {
                speed = 4;
            }

            if(speed == 1) {
                leftPower = Range.clip(gamepad1.left_stick_y, -0.1, 0.1);
                rightPower = Range.clip(gamepad1.right_stick_y, -0.1, 0.1);
            }

            if(speed == 2) {
                leftPower = Range.clip(gamepad1.left_stick_y, -0.25, 0.25);
                rightPower = Range.clip(gamepad1.right_stick_y, -0.25, 0.25);
            }

            if(speed == 3) {
                leftPower = Range.clip(gamepad1.left_stick_y, -0.5, 0.5);
                rightPower = Range.clip(gamepad1.right_stick_y, -0.5, 0.5);
            }

            if(speed == 4) {
                leftPower = Range.clip(gamepad1.left_stick_y, -1.0, 1.0);
                rightPower = Range.clip(gamepad1.right_stick_y, -1.0, 1.0);
            }

            motor1.setPower(leftPower);
            motor2.setPower(rightPower);



            telemetry.update();
        }

    }
}





















