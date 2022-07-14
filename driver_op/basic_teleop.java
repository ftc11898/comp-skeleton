package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

@TeleOp public class BasicTeleOp extends OpMode {

    // Initialize all motors and servos that will be used
    public DcMotor rightFront;  // this is a standard thing that is needed
    public DcMotor leftFront;   // it basically sets up the program
    public DcMotor rightBack;   // its for wiring, like so u know how many motors are being used
    public DcMotor leftBack;   // and like how many servos
    public DcMotor slider;      // just a routine, no code to learn here :)
    public DcMotor intakeone;
    public DcMotor intaketwo; 
    public Servo servoup;
    public Servo servotwo;
    public Servo servothree;
    public Servo servofour;
    
    ModernRoboticsI2cRangeSensor rangeSensor;
    OpticalDistanceSensor odsSensor;  // Hardware Device Object

    @Override
    public void init() {

	// Assign each motor/servo to a device on the hardwareMap (configured on the robot phone)
        rightFront = hardwareMap.dcMotor.get("rightFront");  // now we are assigning a name to each motor/servo
        leftFront = hardwareMap.dcMotor.get("leftFront");   // also nothing special here
        rightBack = hardwareMap.dcMotor.get("rightBack");   // 
        leftBack = hardwareMap.dcMotor.get("leftBack");
       
        slider = hardwareMap.dcMotor.get("slider");

        intakeone = hardwareMap.dcMotor.get("intakeone");
        intaketwo = hardwareMap.dcMotor.get("intaketwo");
        
        servoup = hardwareMap.servo.get("servoup");
        servotwo = hardwareMap.servo.get("servotwo");
        servothree = hardwareMap.servo.get("servothree");
        servofour = hardwareMap.servo.get("servofour");

	// Optional, change the behavior of motors when set to zero power (lock or free spin)
        intakeone.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT); //Slow down intake wheels to a stop
        intaketwo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        
        rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "sensor_range");
        odsSensor = hardwareMap.get(OpticalDistanceSensor.class, "sensor_ods");

        // Optional, change motors to run using potentiometer (positional encoding) instead of voltage
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    
    @Override
    public void start(){
	// Do anything that must be done upon pressing "INITIALIZE" button at beginning of match (move servos into bounding box, etc)
	servotwo.setPosition(.5);
	servothree.setPosition(.2);	
    }

    @Override
    public void loop() {
	/* Assigning game controller messages to commands. Will execute from "START" button to "STOP" */
	    
	// Assigning 1st player joysticks and triggers to drive commands
	// NOTE! we combine forward/back, left/right, and strafing into one command per motor - pay attention to signs (pos/negative)
	
	// right stick y : forward and back
	// right stick x : turning left and right
	// triggers : strafing left and right
	// left stick x and y: diagonals

	leftBack.setPower(.4*(-gamepad1.right_stick_y)+ .6*gamepad1.right_stick_x + .7*(-gamepad1.left_stick_x));
	leftFront.setPower(.4*(-gamepad1.right_stick_y)+ .6*gamepad1.right_stick_x + .7*(gamepad1.left_stick_x));
	rightBack.setPower(.4*(gamepad1.right_stick_y)+ .6*gamepad1.right_stick_x + .7*(-gamepad1.left_stick_x));
	rightFront.setPower(.4*(gamepad1.right_stick_y)+ .6*gamepad1.right_stick_x + .7*(gamepad1.left_stick_x));

	// Assigning 2nd player gamepad buttons to servos

	if (gamepad2.b){ // SERVO LEVEL
	    servoup.setPosition(.2);
	    servotwo.setPosition(.8);
	    servothree.setPosition(.8);
	    servofour.setPosition(.2);
	}

	if (gamepad2.y){ // SERVO GO UP
	    servoup.setPosition(.8);
	    servotwo.setPosition(.2);
	    servothree.setPosition(.2);
	    servofour.setPosition(.8);
	}

	if(gamepad2.a){ // SERVO GO DOWN
	    servoup.setPosition(0);
	    servotwo.setPosition(1);
	    servothree.setPosition(1);
	    servofour.setPosition(0);
	}

	// Assign 2nd player triggers to control linear slider motor 
	    slider.setPower(gamepad2.right_trigger - gamepad2.left_trigger);

	// Assign 2nd player right joystick to intake motors
	    intakeone.setPower(gamepad2.right_stick_y*-.9);
	    intaketwo.setPower(gamepad2.right_stick_y*.9);
    }

    @Override
    public void stop () {
	// Needed to stop robot at the end of the match
    }
}
