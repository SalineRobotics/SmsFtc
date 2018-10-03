/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

//import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import android.hardware.TriggerEventListener;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

/**
 * This file provides basic Telop driving for a Pushbot 
 * The code is structured as an Iterative OpMode
 *
 * This OpMode uses the common Pushbot hardware class to define the devices on the 
 * All device access is managed through the HardwarePushbot class.
 *
 * This particular OpMode executes a basic Tank Drive Teleop for a PushBot
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="BOT10644", group="00")

public class BOT10644 extends OpMode{

    public DcMotor  leftDrive   = null;
    public DcMotor  rightDrive  = null;
    public DcMotor  lCol     = null;
    public DcMotor  rCol    = null;
    public DcMotor  lift     = null;
    public DcMotor  dump    = null;
    public DcMotor  arm = null;

    public Servo    relicArm   = null;
    public Servo    relicHand    = null;
    public Servo    jewelServo = null;
    public Servo spankServo = null;

    public GyroSensor revGYRO = null;
    public DigitalChannel digitalTouch;  // Hardware Device Object
    public static final double MID_SERVO       =  0.5 ;
    public double SERVO_OFFSET = 0.0;
    public static final double ARM_UP_POWER    = -0.45 ;
    public static final double ARM_DOWN_POWER  = 0.45 ;

    double Offset = 0.0;                                                 // could also use HardwarePushbotMatrix class.
    double clawOffset  = 0.0 ;
    double handOffset = 0.0;// Servo mid position
    final double    CLAW_SPEED  = 0.005;                 // sets rate to move servo
    boolean PreviousDrive = false;
    boolean PreviousArm = false;
    boolean turn_speed = false;
    boolean driveDirection = true;
    double leftcol = 0.0;
    double rgthcol = 0.0;
    double driveMode = 0.0;
    double armMode = 0.0;
    double left;
    double right;
    double turn;
    double minClaw = -0.1;
    double maxClaw= 0.30;
    double maxSpeed = 0.5;
    double abc = 0.0;
    double relicArmPower = 0.0;
    double liftPowerTest = 0.0;
    double dumpPowerTest = 0.0;
    boolean previousDPD = false;
    boolean previousDPU = false;
    boolean previousDPL = false;
    boolean previousDPR = false;
    double arm_power = 0.0;
    double mainArmPower = 0;
    boolean clawrprevious = false;
    boolean clawlprevious = false;
    final double joyscale = 127;
    int servo = 1;
    boolean previousX = false;
    boolean previousB = false;
    double intRunTime = 0;

    ElapsedTime runtime = new ElapsedTime();

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        double max = 0.0;

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        try
        {
            dump = hardwareMap.get(DcMotor.class, "dump");
            dump.setDirection(DcMotor.Direction.FORWARD);
            dump.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            dump.setZeroPowerBehavior(BRAKE);
            dump.setPower(0);
            max = -1.0;
        }
        catch (Exception p_exception) { max = 1.0; };

        try {
            digitalTouch = hardwareMap.digitalChannel.get("sensor_digital");
        } catch (Exception p_exception) {  };

        try
        {
            leftDrive = hardwareMap.dcMotor.get("left");
            if (max == 1.0) { SERVO_OFFSET=-0.2;leftDrive.setDirection(DcMotor.Direction.REVERSE); } else { leftDrive.setDirection(DcMotor.Direction.FORWARD); }
            leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        catch (Exception p_exception) { };

        try
        {
            rightDrive = hardwareMap.dcMotor.get("right");
            if (max == 1.0) { rightDrive.setDirection(DcMotor.Direction.FORWARD) ; } else { rightDrive.setDirection(DcMotor.Direction.REVERSE); }
            rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        catch (Exception p_exception) { };

        try
        {
            lCol = hardwareMap.get(DcMotor.class, "leftcol");
            lCol.setDirection(DcMotor.Direction.REVERSE);
            lCol.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            lCol.setPower(0);
        }

        catch (Exception p_exception) { };
        try
        {
            rCol = hardwareMap.get(DcMotor.class, "rightcol");
            //rCol.setDirection(DcMotor.Direction.REVERSE);
            rCol.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rCol.setPower(0);
        }
        catch (Exception p_exception) { };
        try
        {
            arm = hardwareMap.get(DcMotor.class, "arm");
            arm.setDirection(DcMotor.Direction.REVERSE);
            arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            arm.setZeroPowerBehavior(BRAKE);
            arm.setPower(0);
        }
        catch (Exception p_exception) { };

        try
        {
            lift = hardwareMap.get(DcMotor.class, "lift");
            lift.setDirection(DcMotor.Direction.REVERSE);
            lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            lift.setZeroPowerBehavior(BRAKE);
            lift.setPower(0);
        }
        catch (Exception p_exception) { };

        try
        {
            jewelServo  = hardwareMap.get(Servo.class, "jewel");
            jewelServo.setPosition(0.2);
        }
        catch (Exception p_exception) { };

        try
        {
            spankServo  = hardwareMap.get(Servo.class, "spanker");
            spankServo.setPosition(1.0);
        }
        catch (Exception p_exception) { };

        try
        {
            relicHand  = hardwareMap.get(Servo.class, "hand");
            relicHand.setPosition(MID_SERVO);
        }
        catch (Exception p_exception) { };


        try
        {
            relicArm = hardwareMap.get(Servo.class, "scissor");
            relicArm.setPosition(MID_SERVO);
        }
        catch (Exception p_exception) { };


        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */


    @Override
    public void start() {
        runtime.reset();
        //lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //dump.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //dump.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        // Servos
        Offset = 0.0;
        // Use gamepad left & right Bumpers to open and close the claw
        if (gamepad2.y || gamepad2.x || gamepad2.right_bumper) {
            Offset = CLAW_SPEED;
        }
        if (gamepad2.a || gamepad2.b || gamepad2.left_bumper) {
            Offset = -CLAW_SPEED;
        }
            clawOffset += Offset;

        if (gamepad2.dpad_up) { handOffset += CLAW_SPEED; }
        if (gamepad2.dpad_down) {handOffset -= CLAW_SPEED; }

        // Move both servos to new position.  Assume servos are mirror image of each other.
        clawOffset = Range.clip(clawOffset, 0.0, 1.0);
        handOffset = Range.clip(handOffset, 0.0, 1.0);

        if (relicArm != null) {relicArm.setPosition(clawOffset);}
        if (relicHand != null) {relicHand.setPosition(handOffset);}


        left = -gamepad1.left_stick_y * joyscale;
        turn = -gamepad1.right_stick_x * joyscale;
        right = -gamepad1.right_stick_y* joyscale;

        if (Math.abs(left) > 5) {
            left= ((Math.pow(left,2) * 80)/Math.pow(joyscale,2) + 20) * left/Math.abs(left) / 100;
        } else {
            left = 0;
        }

        if (Math.abs(right) > 5) {
            right= ((Math.pow(right,2) * 80)/Math.pow(joyscale,2) + 20) * right/Math.abs(right) / 100;
        } else {
            right = 0;
        }

        if (Math.abs(turn) > 5) {
            turn = ((Math.pow(turn,2) * 80)/Math.pow(joyscale,2) + 20) * turn/Math.abs(turn) / 100;
        } else {
            turn = 0;
        }

        // Allow driver to select Tank vs POV by pressing START
        boolean drivetoggle = gamepad1.start;
        if(drivetoggle && (drivetoggle != PreviousDrive)) {
            driveMode = Math.abs(driveMode - 1.0);
        }
        PreviousDrive = drivetoggle;

        // Allow driver to select ARM CONTROLS
        //boolean armtoggle = gamepad2.start;
        //if(armtoggle && (armtoggle != PreviousArm)) {
        //    armMode = Math.abs(armMode - 1.0);
        //}
        //PreviousArm = armtoggle;
//armMode = 0.0;
        leftcol = 0.0;
        rgthcol = 0.0;

        if (gamepad2.dpad_up) { leftcol =1.0; rgthcol = 1.0; }
        if (gamepad2.dpad_down) { leftcol = -1.0; rgthcol = -1.0; }
        //if (gamepad2.dpad_left) { armMode = -1.0; }
        if (gamepad2.dpad_left) {leftcol =0.5; rgthcol = 0.0;}
        //|| gamepad2.dpad_right ) { armMode = 0.0; }
        if (gamepad2.dpad_right) {leftcol =0.0; rgthcol = 0.5;}


        if (spankServo != null) {
            if (gamepad2.y) {
                spankServo.setPosition(0.0);
            } else {
                spankServo.setPosition(1.0);
            }
        }

        maxSpeed = 0.6;
        if (gamepad1.left_bumper) maxSpeed = 0.3;
        if (gamepad1.right_bumper) maxSpeed = 1.0;
        if (gamepad2.back) { maxSpeed = maxSpeed * -1;}

        if (lCol != null) lCol.setPower(leftcol); // 2018-05-06 changed from maxSpeed * armMode
        if (rCol != null) rCol.setPower(rgthcol);


                right = left + turn * 0.95;
        left = left - turn * 0.95;

        // Normalize the values so neither exceed +/- 1.0
        if (Math.abs(left)>maxSpeed) { left = left/Math.abs(left) * maxSpeed;};
        if (Math.abs(right)>maxSpeed) { right = right/Math.abs(right) * maxSpeed;};

        if (gamepad1.dpad_left) { left = -0.25; right = 0.25; };
        if (gamepad1.dpad_right) { left = 0.25; right = -0.25; };
        if (gamepad1.dpad_down) { left = -0.25; right = -0.25; };
        if (gamepad1.dpad_up) { left = 0.25; right = 0.25; };

        // driveMode now toggles between glyhp/relic hand being the front
        if (driveMode==0) {
        	// reverse sign AND reverse side
        	// need a placeholder
        	turn = left;
        	left = - right;
        	right = -turn;
        } 

        // Arm Motors
        mainArmPower = -gamepad2.left_stick_y * (0.44 - armMode * 0.1);
        if (armMode == 1) { mainArmPower = -mainArmPower;}

        // We turn off the brake to let it float down gently
        /*if (runtime.milliseconds() >= 60 * 1000) {
            mainArmPower += 0.15; // Brake is not powerful enough when fully extended
            maxClaw = 0.5;
            if (gamepad2.x) {
                mainArmPower = 0.0;
                mainArm.setZeroPowerBehavior(FLOAT);
            } else {
                mainArm.setZeroPowerBehavior(BRAKE);
            }
        }
        */
        relicArmPower = -gamepad2.right_stick_y * 0.7 * armMode;

        if (digitalTouch != null) {
            if (digitalTouch.getState() == false) {
                if ( mainArmPower < 0) {mainArmPower = 0.0; }
                telemetry.addData("Digital Touch", "Is Pressed");
            } else {
                telemetry.addData("Digital Touch", "Is Not Pressed");
            }
        }

        // Set Power/Positions
        if (leftDrive != null) leftDrive.setPower(left);
        if (rightDrive != null) rightDrive.setPower(right);
        //if (lift !=null) lift.setPower(mainArmPower);
        //if (dump !=null) dump.setPower(relicArmPower);
        // Temp Lift Code to Find Encoder Values
        liftPowerTest = (Range.clip(gamepad2.left_stick_y, -0.2, 0.2));

        //if (arm != null) mainArm.setPower(mainArmPower);
        //if (relicArm != null) relicArm.setPower(relicArmPower);

        //if (jewelServo != null) jewelServo.setPosition(.95);
        //if (leftClaw != null) {leftClaw.setPosition(MID_SERVO + (clawOffset+SERVO_OFFSET));}
        //if (rightClaw != null) {rightClaw.setPosition(MID_SERVO - (clawOffset+SERVO_OFFSET));}
        //if (relichand != null) {relichand.setPosition(MID_SERVO + handOffset);}

        dumpPowerTest = (Range.clip(gamepad2.right_stick_y, -0.2, 0.2));
        lift.setPower(liftPowerTest);
        dump.setPower(dumpPowerTest);

        telemetry.addData("Lift Motor Position:", lift.getCurrentPosition());
        telemetry.addData("Dump Motor Position:", dump.getCurrentPosition());
        telemetry.addData("Lift Power:", liftPowerTest);
        telemetry.addData("Dump Power:", dumpPowerTest);

        // Send telemetry message to signify robot running;
        telemetry.addData("Servo", "%.1f", clawOffset);
        telemetry.addData("Hand", "%.3f", MID_SERVO + handOffset);
        //telemetry.addData("claw",  "Offset = %.2f", clawOffset);
        telemetry.addData("left",  "%.2f", left);
        telemetry.addData("right", "%.2f", right);
        if (driveMode == 0) { telemetry.addData("POV","", driveMode); } else { telemetry.addData("TANK","", driveMode); }
        if (driveDirection == false) { telemetry.addData("BACKWARDS", "", driveDirection); } else {telemetry.addData("FORWARD", ""); }
        //if (armMode == 0) { telemetry.addData("JOYSTICK","", driveMode); } else { telemetry.addData("D-PAD","", driveMode); }
        //telemetry.addData("claw", "Offset = %.2f", handOffset);
        //telemetry.addData("claw", "Pos = %.2f", relichand.getPosition());

    }



    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}
