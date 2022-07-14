//RELIC RED SIDE 2/14/18
//2/10/18 MR. RANGE SENSOR GOAT CODE

/**
 * Legacy code from 11898's second FTC season ever! We didn't really know the basic principles
 * of good programming design etc so this is all very messy + badly tabbed
 * Was one of four different programs we wrote (one for each corner of the field lol, not advised)
 * Contains examples of VuMark handling, positional encoder motor control, Modern Robotics optical dist + gyro
 * 
 * Just here in case you ever ~really~ need to dig through the weeds - welcome to the chaos
**/

/* Copyright (c) 2017 FIRST. All rights reserved.
 * HELLO
 */
package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import java.text.DecimalFormat;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.ServoControllerEx;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * This OpMode illustrates the basics of using the Vuforia engine to determine
 * the identity of Vuforia VuMarks encountered on the field. The code is structured as
 * a LinearOpMode. It shares much structure with {@link ConceptVuforiaNavigation}; we do not here
 * duplicate the core Vuforia documentation found there, but rather instead focus on the
 * differences between the use of Vuforia for navigation vs VuMark identification.
 *
 * @see ConceptVuforiaNavigation
 * @see VuforiaLocalizer
 * @see VuforiaTrackableDefaultListener
 * see  ftc_app/doc/tutorial/FTC_FieldCoordinateSystemDefinition.pdf
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained in {@link ConceptVuforiaNavigation}.
 */
@Autonomous(name="RelicRED", group ="Concept")
public class RelicSideRED extends LinearOpMode {
    
    public DcMotor rightFront;  // this is a standard thing that is needed
    public DcMotor leftFront;   // it basically sets up the program
    public DcMotor rightBack;   // its for wiring, like so u know how many motors are being used
    public DcMotor leftBack;
    public Servo servoup;
    public Servo servotwo;
    public DcMotor intakeone;
    public DcMotor intaketwo;    private ElapsedTime     runtime = new ElapsedTime();
    public DcMotor slider;
    public DcMotor slider2;
    public Servo servothree;
    public Servo servofour;
    public Servo servocolor;
    public Servo servoball;
    public ModernRoboticsI2cColorSensor color;
    ModernRoboticsI2cRangeSensor rangeSensor;

    
 ModernRoboticsI2cGyro   gyro    = null;                    // Additional Gyro device
    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);
    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.
    static final double     DRIVE_SPEED             = 0.65; //.4    // Nominal speed for better accuracy.
    static final double     TURN_SPEED              = 0.12; //.12
    
    static final double     RIGHT_DISTANCE           = 12;
    static final double     CENTER_DISTANCE           = 16.45;
    static final double     LEFT_DISTANCE           = 19.45; //21.5 //19.45
    // Nominal half speed for better accuracy.
    static final double     HEADING_THRESHOLD       = 1 ;      // As tight as we can make it with an integer gyro
    static final double     P_TURN_COEFF            = 0.15;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_COEFF           = 0.025;     // Larger is more responsive, but also less stable
    
    //IntegratingGyroscope gyro;
    ModernRoboticsI2cGyro modernRoboticsI2cGyro;
    ElapsedTime timer = new ElapsedTime();
    
    public static final String TAG = "Vuforia VuMark Sample";
    OpenGLMatrix lastLocation = null;
    
    
    static final double     COUNTS_PER_INCH_HEIGHT  = 1120;
    static final double     DRAWER_SPEED            = .5;
    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    VuforiaLocalizer vuforia;
    @Override public void runOpMode() {
        boolean lastResetState = false;
        boolean curResetState  = false;
        
        rightFront = hardwareMap.dcMotor.get("rightFront");  // now we are assigning a name to each motor/servo
        leftFront = hardwareMap.dcMotor.get("leftFront");   // also nothing special here
        rightBack = hardwareMap.dcMotor.get("rightBack");   // 
        leftBack = hardwareMap.dcMotor.get("leftBack");
        servoup = hardwareMap.servo.get("servoup");
        servotwo = hardwareMap.servo.get("servotwo");
        intakeone = hardwareMap.dcMotor.get("intakeone");
        intaketwo = hardwareMap.dcMotor.get("intaketwo");        
        slider = hardwareMap.dcMotor.get("slider");
        slider2 = hardwareMap.dcMotor.get("slider2");
        
        servothree = hardwareMap.servo.get("servothree");
        servofour = hardwareMap.servo.get("servofour");
        servocolor = hardwareMap.servo.get("servocolor");
        servoball = hardwareMap.servo.get("servoball");
        color = hardwareMap.get(ModernRoboticsI2cColorSensor.class, "color" );
        ElapsedTime eTime = new ElapsedTime();

        rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "sensor_range");

        gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");
        
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slider2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slider2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT); //Slow down intake wheel one to a stop
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT); //Slow down intake wheel one to a stop
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        /*
         * To start up Vuforia, tell it the view that we wish to use for camera monitor (on the RC phone);
         * If no camera monitor is desired, use the parameterless constructor instead (commented out below).
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        
      
        // OR...  Do Not Activate the Camera Monitor View, to save power
         //VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        /*
         * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
         * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
         * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
         * web site at https://developer.vuforia.com/license-manager.
         *
         * Vuforia license keys are always 380 characters long, and look as if they contain mostly
         * random data. As an example, here is a example of a fragment of a valid key:
         *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
         * Once you've obtained a license key, copy the string from the Vuforia web site
         * and paste it in to your code onthe next line, between the double quotes.
         */
        parameters.vuforiaLicenseKey = "ARiZWML/////AAAAmbcUlZg6e0L1rJYYv8gPJvkR2oYi9ypFwFc0svzxH2Id1XgyiIAlFfDEW9C4G1AmkWn0DjNwN2zbC/pNbzjUEN3ZxqWG3VX+LnhupICBAVhYbXL5nFfUYwgeHNjhf30Im5ylKzQNGZh+b7dkHofSfs8690GVW3W88OcakX6B4POTDouvyWEHMOfi1UDNGOGNlAznrBYK/6XnWbR5hhNIAYeLE+iQUQI3zgz7ZveDsYsYkiTtWGQqMvLSoHrJtTrpMvtwwPN+DWxf2+X2I3y2eA4Yvz+TdJiHqGJLVunqqYfkSaKfMw1noW1wB2cuwxKTp+AQAU//xabfJzqiGYbDlLgdUH5sZ4OsoJtFlBXzlurx";
        /*
         * We also indicate which camera on the RC that we wish to use.
         * Here we chose the back (HiRes) camera (for greater range), but
         * for a competition robot, the front camera might be more convenient.
         */
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        /**
         * Load the data set containing the VuMarks for Relic Recovery. There's only one trackable
         * in this data set: all three of the VuMarks in the game were created from this one template,
         * but differ in their instance id information.
         * @see VuMarkInstanceId
         */
         
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary
        telemetry.addData(">", "Press Play to start");
        telemetry.update();
        
        
        // Send telemetry message to alert driver that we are calibrating;
        telemetry.addData(">", "Calibrating Gyro");    //
        telemetry.update();
        

        gyro.calibrate();
        // make sure the gyro is calibrated before continuing
        while (!isStopRequested() && gyro.isCalibrating())  {
            sleep(50);
            idle();
        }
        
        telemetry.addData(">", "Robot Ready.");    //
        telemetry.update();

        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        // Wait for the game to start (Display Gyro value), and reset gyro before we move..
        
        while (!isStarted()) {
            telemetry.addData(">", "Robot Heading = %d", gyro.getIntegratedZValue());
            telemetry.update();
        }
        gyro.resetZAxisIntegrator();
        
        waitForStart();
        relicTrackables.activate();
        color.enableLed(true);
        servocolor.setPosition(0.5);
        sleep(500);
        servoball.setPosition(1);
        servoup.setPosition(0);
        servotwo.setPosition(1);
        servothree.setPosition(1);
        servofour.setPosition(0);
        
        
        while (opModeIsActive() && !isStopRequested()) {
            telemetry.addData("raw ultrasonic", rangeSensor.rawUltrasonic());
            telemetry.addData("raw optical", rangeSensor.rawOptical());
            telemetry.addData("cm optical", "%.2f cm", rangeSensor.cmOptical());
            telemetry.addData("cm", "%.2f cm", rangeSensor.getDistance(DistanceUnit.CM));
            telemetry.update();
        
            telemetry.addData("Color Number",color.readUnsignedByte(ModernRoboticsI2cColorSensor.Register.COLOR_NUMBER));
            telemetry.update();
            sleep(90);
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            sleep(500);
            eTime.reset();
            while(eTime.time()<1){
                if (color.readUnsignedByte(ModernRoboticsI2cColorSensor.Register.COLOR_NUMBER) == 3){
                    servocolor.setPosition(0.6);
                    sleep(1000);
                    servocolor.setPosition(0.5);
                    servoball.setPosition(.2);
                }
                if (color.readUnsignedByte(ModernRoboticsI2cColorSensor.Register.COLOR_NUMBER) == 10){
                    servocolor.setPosition(.3);
                    sleep(1000);
                    servocolor.setPosition(.5);
                    servoball.setPosition(.2);
                }
                //else{
                    
                  //  servoball.setPosition(.95);
                //}
            }
            servoball.setPosition(.2);

             if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
                /* In actual game, will loop until condition occurs, then act based on 
                 * which VuMark was visible. */
                telemetry.addData("VuMark", "%s visible", vuMark);
                if(vuMark == RelicRecoveryVuMark.CENTER) {
                  //  encoderDrive(DRIVE_SPEED,  -22.6,  -22.6);  // S1: Forward X Inches
                    
                    //encoderDrive(DRIVE_SPEED, CENTER_DISTANCE, -CENTER_DISTANCE);
                    
                    rightFront.setDirection(DcMotor.Direction.REVERSE);
                    rightBack.setDirection(DcMotor.Direction.REVERSE);
                    gyroDrive(.3,-CENTER_DISTANCE,0);    //DRIVE OFF STONE
                    gyroTurn(TURN_SPEED, -98); // TURN 98 DEGREES TO FACE THE GLYPHS 
                    //gyroDrive(.1, 1.5,-90);
                    
                    gyroDrive(.1, -6.5, -98);// BACK UP TO SCORE FIRST GLYPH
                    
                    while(rangeSensor.getDistance(DistanceUnit.CM)>24){
                        leftBack.setPower(-.1);
                        leftFront.setPower(-.1);
                        rightBack.setPower(-.1);
                        rightFront.setPower(-.1);
                    }
                    leftBack.setPower(0);
                    leftFront.setPower(0);
                    rightFront.setPower(0);
                    rightBack.setPower(0);
                    delay(1.0);
                
                    gyroTurn(TURN_SPEED, -98); // A TURN TO MAKE SURE STILL FACING -98
                    
                    while(rangeSensor.getDistance(DistanceUnit.CM)>17.5){ // STRAFE UNTIL THRESHOLD
                        leftFront.setPower(.1);
                        leftBack.setPower(-.1);
                        rightFront.setPower(-.1);
                        rightBack.setPower(.1);
                        telemetry.addData("cm", "%.2f cm", rangeSensor.getDistance(DistanceUnit.CM));
                        telemetry.update();
                    }
                    leftFront.setPower(0);  // STOP THE STRAFING
                    leftBack.setPower(0);
                    rightFront.setPower(0);
                    rightBack.setPower(0);
                    delay(1.0); // WAIT A SECOND IN BETWEEN
                    

                    servoup.setPosition(.8); // SERVOPLATE UP, DONT CHANGE THIS
                    servotwo.setPosition(.2);
                    servothree.setPosition(.2);
                    servofour.setPosition(.8);
                    
                    delay(1.0);   // WAIT A SECOND IN BETWEEN
                    
                    gyroDrive(.5, 3, -98);
               
                    servoup.setPosition(0); //SERVOPLATE DOWN , DONT CHANGE THIS
                    servotwo.setPosition(1);
                    servothree.setPosition(1);
                    servofour.setPosition(0);
          
                    intakeone.setPower(-1);  // TURN ON COLLECTOR
                    intaketwo.setPower(1);
                    
                    
                    gyroTurn(.2,-95); // TURN 95 DEGREES, BEFORE GOING FOR GLYPHS
                    
                    
                    gyroDrive(.5, 16,-95); //DRIVE 16 INCHES TO GLYPHS AT .5 SPEED
                   
                    delay(1.0);

              

                    gyroTurn(.2,-98);  // NOW TURN 98 DEGREES (HORIZONTAL AGAIN)
                    
                    gyroDrive(DRIVE_SPEED, 5, -98); //BURST FORWARDS TO COLLECT GLYPH

                    gyroDrive(.5, -26, -98);  //DRIVE BACK 22.5 INCHES AT .2 SPEED
                    while(rangeSensor.getDistance(DistanceUnit.CM)>24){
                        leftBack.setPower(-.1);
                        leftFront.setPower(-.1);
                        rightBack.setPower(-.1);
                        rightFront.setPower(-.1);
                    }
                    leftBack.setPower(0);
                    leftFront.setPower(0);
                    rightFront.setPower(0);
                    rightBack.setPower(0);
                    delay(1.0);
                    
                    gyroTurn(TURN_SPEED,-100); // TURN A BIT MORE ONCE U STOP , B4 STRAFING , KEEP AT 100

                    while(rangeSensor.getDistance(DistanceUnit.CM)>17.5){ //STRAFE
                        leftFront.setPower(.1);
                        leftBack.setPower(-.1);
                        rightFront.setPower(-.1);
                        rightBack.setPower(.1);
                        telemetry.addData("cm", "%.2f cm", rangeSensor.getDistance(DistanceUnit.CM));
                        telemetry.update();
                    }
                
                    leftFront.setPower(0);
                    leftBack.setPower(0);
                    rightFront.setPower(0);
                    rightBack.setPower(0);
                    
                    delay(1);
     
                
                    
                    gyroTurn(TURN_SPEED,-100); // CHECK TO SEE IF U 98 AGAIN
            

                    delay(0.7);
                    servoup.setPosition(.8);
                    servotwo.setPosition(.2);
                    servothree.setPosition(.2);
                    servofour.setPosition(.8);

                    delay (1);
                    gyroDrive(.2,2,-100); //DRIVE FORWARD BEFORE U SERVO PLATE DOWN
                    
                    
                    servoup.setPosition(0);
                    servotwo.setPosition(1);
                    servothree.setPosition(1);
                    servofour.setPosition(0);

                    break;
                    
                } else if(vuMark == RelicRecoveryVuMark.RIGHT){

                    
                    rightFront.setDirection(DcMotor.Direction.REVERSE);
                    rightBack.setDirection(DcMotor.Direction.REVERSE);
                    gyroDrive(.3,-RIGHT_DISTANCE,0);    //DRIVE OFF STONE
                    gyroTurn(TURN_SPEED, -98); // TURN 98 DEGREES TO FACE THE GLYPHS 
                    //gyroDrive(.1, 1.5,-90);
                    
                    gyroDrive(.1, -6.5, -98);// BACK UP TO SCORE FIRST GLYPH
                
                    gyroTurn(TURN_SPEED, -98); // A TURN TO MAKE SURE STILL FACING -98
                    
                    while(rangeSensor.getDistance(DistanceUnit.CM)>17.5){ // STRAFE UNTIL THRESHOLD
                        leftFront.setPower(.1);
                        leftBack.setPower(-.1);
                        rightFront.setPower(-.1);
                        rightBack.setPower(.1);
                        telemetry.addData("cm", "%.2f cm", rangeSensor.getDistance(DistanceUnit.CM));
                        telemetry.update();
                    }
                    leftFront.setPower(0);  // STOP THE STRAFING
                    leftBack.setPower(0);
                    rightFront.setPower(0);
                    rightBack.setPower(0);
                    delay(1); // WAIT A SECOND IN BETWEEN
                    

                    servoup.setPosition(.8); // SERVOPLATE UP, DONT CHANGE THIS
                    servotwo.setPosition(.2);
                    servothree.setPosition(.2);
                    servofour.setPosition(.8);
                    
                    delay(1.0);   // WAIT A SECOND IN BETWEEN
                    
                    //gyroDrive(.5, 3, -98);
               
                    servoup.setPosition(0); //SERVOPLATE DOWN , DONT CHANGE THIS
                    servotwo.setPosition(1);
                    servothree.setPosition(1);
                    servofour.setPosition(0);
          
                    intakeone.setPower(-1);  // TURN ON COLLECTOR
                    intaketwo.setPower(1);
                    
                    
                    gyroTurn(.2,-95); // TURN 95 DEGREES, BEFORE GOING FOR GLYPHS
                    
                    
                    gyroDrive(.5, 16,-95); //DRIVE 16 INCHES TO GLYPHS AT .5 SPEED
                   
                    delay(0.8);

              

                    gyroTurn(.2,-98);  // NOW TURN 98 DEGREES (HORIZONTAL AGAIN)
                    
                    gyroDrive(DRIVE_SPEED, 5, -98); //BURST FORWARDS TO COLLECT GLYPH

                    gyroDrive(.5, -23, -98);  //DRIVE BACK 22.5 INCHES AT .2 SPEED
                    
                    gyroTurn(TURN_SPEED,-100); // TURN A BIT MORE ONCE U STOP , B4 STRAFING , KEEP AT 100

                    while(rangeSensor.getDistance(DistanceUnit.CM)>17.5){ //STRAFE
                        leftFront.setPower(.1);
                        leftBack.setPower(-.1);
                        rightFront.setPower(-.1);
                        rightBack.setPower(.1);
                        telemetry.addData("cm", "%.2f cm", rangeSensor.getDistance(DistanceUnit.CM));
                        telemetry.update();
                    }
                
                    leftFront.setPower(0);
                    leftBack.setPower(0);
                    rightFront.setPower(0);
                    rightBack.setPower(0);
                    
                    delay(1);
     
                
                    
                    gyroTurn(TURN_SPEED,-100); // CHECK TO SEE IF U 98 AGAIN
            

                    delay(0.8);
                    servoup.setPosition(.8);
                    servotwo.setPosition(.2);
                    servothree.setPosition(.2);
                    servofour.setPosition(.8);

                    delay (1);
                    gyroDrive(.2,2,-100); //DRIVE FORWARD BEFORE U SERVO PLATE DOWN
                    
                    
                    servoup.setPosition(0);
                    servotwo.setPosition(1);
                    servothree.setPosition(1);
                    servofour.setPosition(0);

                    break;    
                    
                } else {

                    rightFront.setDirection(DcMotor.Direction.REVERSE);
                    rightBack.setDirection(DcMotor.Direction.REVERSE);
                    gyroDrive(.3,-LEFT_DISTANCE,0);    //DRIVE OFF STONE
                    gyroTurn(TURN_SPEED, -98); // TURN 98 DEGREES TO FACE THE GLYPHS 
                    delay(.5);
                    //gyroDrive(.1, 1.5,-90);
                    
                    gyroDrive(.1, -6.5, -98);// BACK UP TO SCORE FIRST GLYPH
                    
                    while(rangeSensor.getDistance(DistanceUnit.CM)>24){
                        leftBack.setPower(-.1);
                        leftFront.setPower(-.1);
                        rightBack.setPower(-.1);
                        rightFront.setPower(-.1);
                    }
                    leftBack.setPower(0);
                    leftFront.setPower(0);
                    rightFront.setPower(0);
                    rightBack.setPower(0);
                    delay(1.0);
                
                    gyroTurn(TURN_SPEED, -98); // A TURN TO MAKE SURE STILL FACING -98
                    
                    while(rangeSensor.getDistance(DistanceUnit.CM)>17.5){ // STRAFE UNTIL THRESHOLD
                        leftFront.setPower(.1);
                        leftBack.setPower(-.1);
                        rightFront.setPower(-.1);
                        rightBack.setPower(.1);
                        telemetry.addData("cm", "%.2f cm", rangeSensor.getDistance(DistanceUnit.CM));
                        telemetry.update();
                    }
                    leftFront.setPower(0);  // STOP THE STRAFING
                    leftBack.setPower(0);
                    rightFront.setPower(0);
                    rightBack.setPower(0);
                    delay(1.0); // WAIT A SECOND IN BETWEEN
                    

                    servoup.setPosition(.8); // SERVOPLATE UP, DONT CHANGE THIS
                    servotwo.setPosition(.2);
                    servothree.setPosition(.2);
                    servofour.setPosition(.8);
                    
                    delay(1.0);   // WAIT A SECOND IN BETWEEN
                    
                    //gyroDrive(.5, 3, -98);
               
                    servoup.setPosition(0); //SERVOPLATE DOWN , DONT CHANGE THIS
                    servotwo.setPosition(1);
                    servothree.setPosition(1);
                    servofour.setPosition(0);
          
                    intakeone.setPower(-1);  // TURN ON COLLECTOR
                    intaketwo.setPower(1);
                    
                    
                    gyroTurn(.2,-95); // TURN 95 DEGREES, BEFORE GOING FOR GLYPHS
                    
                    
                    gyroDrive(.5, 16,-95); //DRIVE 16 INCHES TO GLYPHS AT .5 SPEED
                   
                    delay(1.0);

              

                    gyroTurn(.2,-98);  // NOW TURN 98 DEGREES (HORIZONTAL AGAIN)
                    
                    gyroDrive(DRIVE_SPEED, 5, -98); //BURST FORWARDS TO COLLECT GLYPH

                    gyroDrive(.5, -25, -98);  //DRIVE BACK 22.5 INCHES AT .2 SPEED
                    while(rangeSensor.getDistance(DistanceUnit.CM)>26){
                        leftBack.setPower(-.1);
                        leftFront.setPower(-.1);
                        rightBack.setPower(-.1);
                        rightFront.setPower(-.1);
                    }
                    leftBack.setPower(0);
                    leftFront.setPower(0);
                    rightFront.setPower(0);
                    rightBack.setPower(0);
                    delay(1.0);
                    
                    gyroTurn(TURN_SPEED,-93); // TURN A BIT MORE ONCE U STOP , B4 STRAFING , KEEP AT 100

                    while(rangeSensor.getDistance(DistanceUnit.CM)>19){ //STRAFE
                        leftFront.setPower(.1);
                        leftBack.setPower(-.1);
                        rightFront.setPower(-.1);
                        rightBack.setPower(.1);
                        telemetry.addData("cm", "%.2f cm", rangeSensor.getDistance(DistanceUnit.CM));
                        telemetry.update();
                    }
                
                    leftFront.setPower(0);
                    leftBack.setPower(0);
                    rightFront.setPower(0);
                    rightBack.setPower(0);
                    
                    delay(1);

                    //gyroTurn(TURN_SPEED,-100); // CHECK TO SEE IF U 98 AGAIN
            

                    delay(0.7);
                    servoup.setPosition(.8);
                    servotwo.setPosition(.2);
                    servothree.setPosition(.2);
                    servofour.setPosition(.8);

                    delay (1);
                    gyroDrive(.2,2,-98); //DRIVE FORWARD BEFORE U SERVO PLATE DOWN
                    
                    servoup.setPosition(0);
                    servotwo.setPosition(1);
                    servothree.setPosition(1);
                    servofour.setPosition(0);

                    break;                    
                }
            }
	     
            else {
                telemetry.addData("VuMark", "not visible");
            }
	     
        }
    }

            

    public void flipUp(){
        servoup.setPosition(.8);
        servotwo.setPosition(.2);
        servothree.setPosition(.2);
        servofour.setPosition(.8);
    }
    
    public void flipDown(){
        servoup.setPosition(0);
        servotwo.setPosition(1);
        servothree.setPosition(1);
        servofour.setPosition(0);
    }
    
    public void detectColor(){
        ElapsedTime eTime = new ElapsedTime();
        servoball.setPosition(0.2);
        //color.enableLed(true);//dont know if this will work
        eTime.reset();
        while(eTime.time()<5){
            if (color.readUnsignedByte(ModernRoboticsI2cColorSensor.Register.COLOR_NUMBER) == 3){
                servocolor.setPosition(0.6);
                servocolor.setPosition(0.45);
                servoball.setPosition(.95);
            }
            if (color.readUnsignedByte(ModernRoboticsI2cColorSensor.Register.COLOR_NUMBER) == 10){
                servocolor.setPosition(.3);
                servocolor.setPosition(.45);
                servoball.setPosition(.95);
            }
            //else{
                
              //  servoball.setPosition(.95);
            //}
        }
    }
    
    public void encoderDrive(double speed, double leftInches, double rightInches) {
        int newLeftTarget;
        int newRightTarget;
        int newLeftTargetB;
        int newRightTargetB;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = leftFront.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = rightFront.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            newLeftTargetB = leftBack.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTargetB = rightBack.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            leftFront.setTargetPosition(newLeftTarget);
            rightFront.setTargetPosition(newRightTarget);
            leftBack.setTargetPosition(newLeftTargetB);
            rightBack.setTargetPosition(newRightTargetB);

            // Turn On RUN_TO_POSITION
            leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            leftFront.setPower(speed);
            rightFront.setPower(speed);
            leftBack.setPower(speed);
            rightBack.setPower(speed);

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                   (leftFront.isBusy() && rightFront.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                                            leftFront.getCurrentPosition(),
                                            rightFront.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            leftFront.setPower(0);
            rightFront.setPower(0);
            leftBack.setPower(0);
            rightBack.setPower(0);

            // Turn off RUN_TO_POSITION
            leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }
	    
    public void delay(double secs) {
        try {
            Thread.sleep((long) secs * 1000);    // just keep this, Please.
        }
	catch (InterruptedException e){
            e.printStackTrace();
        }
    }

    public void gyroDrive ( double speed,
                            double distance,
                            double angle) {

        int     newLeftTarget;
        int     newRightTarget;
        int     moveCounts;
        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            moveCounts = (int)(distance * COUNTS_PER_INCH);
            newLeftTarget = leftFront.getCurrentPosition() + moveCounts;
            newRightTarget = rightFront.getCurrentPosition() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            leftFront.setTargetPosition(newLeftTarget);
            rightFront.setTargetPosition(newRightTarget);
            leftBack.setTargetPosition(newLeftTarget);
            rightBack.setTargetPosition(newRightTarget);

            leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            leftFront.setPower(speed);
            rightFront.setPower(speed);
            leftBack.setPower(speed);
            rightBack.setPower(speed);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                   (leftFront.isBusy() && rightFront.isBusy())) {

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0)
                {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                leftFront.setPower(leftSpeed);
                rightFront.setPower(rightSpeed);
                leftBack.setPower(leftSpeed);
                rightBack.setPower(rightSpeed);
                

                // Display drive status for the driver.
                telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
                telemetry.addData("Target",  "%7d:%7d",      newLeftTarget,  newRightTarget);
                telemetry.addData("Actual",  "%7d:%7d",      leftFront.getCurrentPosition(),
                                                             rightFront.getCurrentPosition(), leftBack.getCurrentPosition(),
                                                             rightBack.getCurrentPosition());
                telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);
                telemetry.addData("cm", "%.2f cm", rangeSensor.getDistance(DistanceUnit.CM));

                telemetry.update();
            }

            // Stop all motion;
            leftFront.setPower(0);
            rightFront.setPower(0);
            leftBack.setPower(0);
            rightBack.setPower(0);

            // Turn off RUN_TO_POSITION
            leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            
        }
    }

    /**
     *  Method to spin on central axis to point in a new direction.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the heading (angle)
     *  2) Driver stops the opmode running.
     *
     * @param speed Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     */
    public void gyroTurn (  double speed, double angle) {

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
        }
    }

    /**
     *  Method to obtain & hold a heading for a finite amount of time
     *  Move will stop once the requested time has elapsed
     *
     * @param speed      Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     * @param holdTime   Length of time (in seconds) to hold the specified heading.
     */
    public void gyroHold( double speed, double angle, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            onHeading(speed, angle, P_TURN_COEFF);
            telemetry.update();
        }

        // Stop all motion;
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
        
    }

    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param speed     Desired speed of turn.
     * @param angle     Absolute Angle (in Degrees) relative to last gyro reset.
     *                  0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                  If a relative angle is required, add/subtract from current heading.
     * @param PCoeff    Proportional Gain coefficient
     * @return
     */
    boolean onHeading(double speed, double angle, double PCoeff) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
            rightSpeed  = speed * steer;
            leftSpeed   = -rightSpeed;
        }

        // Send desired speeds to motors.
        leftFront.setPower(leftSpeed);
        rightFront.setPower(rightSpeed);
        leftBack.setPower(leftSpeed);
        rightFront.setPower(rightSpeed);
        

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     * @param   targetAngle  Desired angle (relative to global reference established at last Gyro Reset).
     * @return  error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     *          +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getError(double targetAngle) {

        double robotError;
        // calculate error in -179 to +180 range  (
        robotError = targetAngle - gyro.getIntegratedZValue();
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     * @param error   Error angle in robot relative degrees
     * @param PCoeff  Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

}
