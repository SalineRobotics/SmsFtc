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
public class ServoTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        //servoTest = hardwareMap.get(, "left");
        Servo servoTest = hardwareMap.servo.get("testservo");
        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a) {
                servoTest.setPosition(0.25);
            }

            if (gamepad1.b) {
                servoTest.setPosition(0.70);
            }
        }

    }
}





















