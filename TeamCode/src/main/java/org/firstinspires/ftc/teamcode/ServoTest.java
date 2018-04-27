
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

import java.util.Date;
import java.util.concurrent.TimeUnit;

@TeleOp
public class ServoTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        //servoTest = hardwareMap.get(, "left");
        Servo servoTest = hardwareMap.servo.get("claw");
        Servo servoTest2 = hardwareMap.servo.get("claw2");
        Servo servoTest3 = hardwareMap.servo.get("claw3");
        double servoPosition = 0.5;
        servoTest.setPosition(servoPosition);
        double          clawOffset      = 0;
        double          clawOffset2     = 0;
        double          clawOffset3     =0;
        boolean previousState = false;
        final double    CLAW_SPEED      = 0.08 ;
        boolean belt = false;
        Date stamp = new Date();
        waitForStart();

        while (opModeIsActive()) {
            Date loopDate = new Date();
            long diff = loopDate.getTime() - stamp.getTime();

            if (diff >= 50) {
                stamp = new Date();
                if (gamepad1.x) {
                    clawOffset += CLAW_SPEED;
                }

                else if (gamepad1.b) {
                    clawOffset -= CLAW_SPEED;
                }

                if (gamepad1.y) {
                    clawOffset2 += CLAW_SPEED;
                }

                else if (gamepad1.a) {
                    clawOffset2 -= CLAW_SPEED;
                }

                if (gamepad1.dpad_up) {
                    clawOffset3 += CLAW_SPEED;
                }

                else if (gamepad1.dpad_down) {
                    clawOffset3 -= CLAW_SPEED;
                }


                // Move both servos to new position.  Assume servos are mirror image of each other.
                clawOffset = Range.clip(clawOffset, -0.5, 0.5);
                clawOffset2 = Range.clip(clawOffset2, -0.5, 0.5);
                servoTest2.setPosition(0.5 + clawOffset2);
                servoTest.setPosition(0.5 + clawOffset);

                telemetry.addData("Servo Position:", servoPosition);
                telemetry.update();

            }
            /*
            if (gamepad1.x) {
                servoPosition = servoPosition + 0.1;
                servoTest.setPosition(servoPosition);
            }

            if (gamepad1.b) {
                servoPosition = servoPosition - 0.1;
                telemetry.addData("Servo Position:", servoPosition);
                telemetry.update();
                servoTest.setPosition(servoPosition);
            }
            */


        }

    }
}





















