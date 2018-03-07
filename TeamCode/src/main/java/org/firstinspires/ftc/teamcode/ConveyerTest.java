
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp
public class ConveyerTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        //servoTest = hardwareMap.get(, "left");
        DcMotor motor1 = hardwareMap.dcMotor.get("motor1");
        DcMotor motor2 = hardwareMap.dcMotor.get("motor2");
        motor1.setDirection(DcMotorSimple.Direction.FORWARD);
        motor2.setDirection(DcMotorSimple.Direction.REVERSE);
        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        boolean belt = false;
        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.start) {
                belt = !belt;
            }
            if (belt) {
                motor1.setPower(0.5);
                motor2.setPower(0.5);
            }

            else {
                motor1.setPower(0);
                motor2.setPower(0);
            }


            telemetry.addData("Belt On/Off:", belt);
            telemetry.update();
        }

    }
}





















