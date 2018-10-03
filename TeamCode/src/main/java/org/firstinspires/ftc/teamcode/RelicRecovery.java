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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.FLOAT;

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

@TeleOp(name="RELIC RECOVERY", group="00")

public class RelicRecovery extends OpMode{

    public DcMotor  leftDrive   = null;
    public DcMotor  rightDrive  = null;
    public DcMotor  mainArm     = null;
    public DcMotor  relicArm    = null;
    public Servo    relichand   = null;
    public Servo    leftClaw    = null;
    public Servo    rightClaw   = null;
    public Servo jewelServo = null;

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
    double driveMode = 0.0;
    double armMode = 1.0;
    double left;
    double right;
    double turn;
    double minClaw = -0.1;
    double maxClaw= 0.30;
    double maxSpeed = 0.5;
    double abc = 0.0;
    double relicArmPower = 0.0;
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
        Servo dummy = null;

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        try {
            dummy = hardwareMap.servo.get("dummy");   max = -1.0;}

        catch (Exception p_exception) { max = 1.0; };

        try {
            digitalTouch = hardwareMap.digitalChannel.get("sensor_digital");
        } catch (Exception p_exception) {  };

        try
        {
            leftDrive = hardwareMap.dcMotor.get("left");
            if (max == 1.0) { SERVO_OFFSET=-0.2;leftDrive.setDirection(DcMotor.Direction.FORWARD); } else { leftDrive.setDirection(DcMotor.Direction.REVERSE); }
            leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        catch (Exception p_exception) { };

        try
        {
            rightDrive = hardwareMap.dcMotor.get("right");
            if (max == 1.0) { rightDrive.setDirection(DcMotor.Direction.REVERSE) ; } else { rightDrive.setDirection(DcMotor.Direction.FORWARD); }
            rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        catch (Exception p_exception) { };

        dummy = null;

        // Define and Initialize Motors

        try
        {
            mainArm = hardwareMap.get(DcMotor.class, "arm");
            mainArm.setDirection(DcMotor.Direction.REVERSE);
            mainArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            mainArm.setZeroPowerBehavior(BRAKE);
            mainArm.setPower(0);
        }
        catch (Exception p_exception) { };

        try
        {
            relicArm = hardwareMap.get(DcMotor.class, "slide");
            relicArm.setDirection(DcMotor.Direction.FORWARD);
            relicArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            relicArm.setZeroPowerBehavior(BRAKE);
            relicArm.setPower(0);
        }
        catch (Exception p_exception) { };


        try
        {
            leftClaw  = hardwareMap.get(Servo.class, "lefthand");
            leftClaw.setPosition(MID_SERVO);
        }
        catch (Exception p_exception) { };

        try
        {
            jewelServo  = hardwareMap.get(Servo.class, "jewel");
            jewelServo.setPosition(0.95);
        }
        catch (Exception p_exception) { };

        try
        {
            relichand  = hardwareMap.get(Servo.class, "claw");
            relichand.setPosition(MID_SERVO);
        }
        catch (Exception p_exception) { };


        try
        {
            rightClaw = hardwareMap.get(Servo.class, "righthand");
            rightClaw.setPosition(MID_SERVO);
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
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

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
        boolean armtoggle = gamepad2.start;
        if(armtoggle && (armtoggle != PreviousArm)) {
            armMode = Math.abs(armMode - 1.0);
        }
        PreviousArm = armtoggle;

        maxSpeed = 0.6;
        if (gamepad1.left_bumper) maxSpeed = 0.3;
        if (gamepad1.right_bumper) maxSpeed = 1.0;

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

        // Servos
        Offset = 0.0;
        // Use gamepad left & right Bumpers to open and close the claw
        if (gamepad2.y || gamepad2.x || gamepad2.right_bumper) {
            Offset = CLAW_SPEED;
        }
        if (gamepad2.a || gamepad2.b || gamepad2.left_bumper) {
            Offset = -CLAW_SPEED;
        }
				if (armMode == 0.0) {
					clawOffset += Offset;
				} else {
				  handOffset += Offset;
				}
				
        // Move both servos to new position.  Assume servos are mirror image of each other.
        clawOffset = Range.clip(clawOffset, minClaw, maxClaw);
        handOffset = Range.clip(handOffset, 0.0, 0.5);
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
        if (mainArm != null) mainArm.setPower(mainArmPower);
        if (relicArm != null) relicArm.setPower(relicArmPower);

        if (jewelServo != null) jewelServo.setPosition(.95);
        if (leftClaw != null) {leftClaw.setPosition(MID_SERVO + (clawOffset+SERVO_OFFSET));}
        if (rightClaw != null) {rightClaw.setPosition(MID_SERVO - (clawOffset+SERVO_OFFSET));}
        if (relichand != null) {relichand.setPosition(MID_SERVO + handOffset);}

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
