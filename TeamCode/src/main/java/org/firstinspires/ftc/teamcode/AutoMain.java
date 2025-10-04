package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


// Telemetry - print on driver hub
//


// Robot Hardware Classes
//
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.VoltageSensor;

// Rev LED Blinkin Classes
//
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

// Driving with Gyro (IMU) classes
//
// import for IMU (gyroscope)
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

// Not sure if needed.
//
import android.graphics.Color;


@Autonomous(name = "AutoMain", group = "")

public class AutoMain extends LinearOpMode {

    private DcMotor FLMoto;
    private DcMotor FRMoto;
    private DcMotor BLMoto;
    private DcMotor BRMoto;

    // arm motors

    private DcMotor stringMoto;
    private DcMotor climbMoto;
    private DcMotor hangMoto;

    // claw servos
    private Servo clawTilt;
    private Servo LClaw;
    private Servo RClaw;
    private Servo FINGER;

    //sensors
    private NormalizedColorSensor color;

    private IMU imu;
    private Orientation lastAngle = new Orientation();

    private VoltageSensor VoltSens;

    //BlinkinLEDs
    private RevBlinkinLedDriver blinkinLedDriver;

    private RevBlinkinLedDriver.BlinkinPattern BasePattern = RevBlinkinLedDriver.BlinkinPattern.VIOLET;
    private RevBlinkinLedDriver.BlinkinPattern StopPattern = RevBlinkinLedDriver.BlinkinPattern.WHITE;
    private RevBlinkinLedDriver.BlinkinPattern RightPattern = RevBlinkinLedDriver.BlinkinPattern.RED;
    private RevBlinkinLedDriver.BlinkinPattern CenterPattern = RevBlinkinLedDriver.BlinkinPattern.BLUE;
    private RevBlinkinLedDriver.BlinkinPattern LeftPattern = RevBlinkinLedDriver.BlinkinPattern.GREEN;


    /*********************/
    /* Variables for imu */
    /*********************/
    double            globalAngle, correction;
    YawPitchRollAngles   lastAngles;

    double voltage = 0;
    double batteryConst = 13.5;
    private static double powerConstant = .5;

    float hsvValues[] = {
            0F,
            0F,
            0F
    };

    final float values[] = hsvValues;
    /***************************/
    /* Possible Automous Modes */
    /***************************/
    public enum AutoMode
    {
        AUTO_MODE_NOT_SELECTED,
        AUTO_MODE_SAMPLE,
        AUTO_MODE_SPECIMEN
    }

    /***************************/
    /* Possible Park Location  */
    /***************************/
    public enum ParkLocation
    {
        PARK_LOC_NOT_SELECTED,
        PARK_LOC_OBS_ZONE_WALL,
        PARK_LOC_OBS_ZONE,
        PARK_LOC_L1_ASCENT_RIGHT,
        PARK_LOC_L1_ASCENT_LEFT
    }


    /***************************************/
    /* Variables for driving with Encoders */
    /***************************************/
    private static final Double wheelCircumference = 4.0 * Math.PI;
    private static final Double gearRatio = 18.9;                  // Rev HD Hex 20:1
    private static final Double countsPerRotation = 560.0;         // Rev HD Hex 20:1
    private static final Double scaleFactor = 17.0;                 // need to find scale factor!!!
    private static final Double straifFactor = 1.7;
    private static final Double countsPerInch = countsPerRotation / wheelCircumference / gearRatio * scaleFactor;


    // Gobal Variables
    private static Double noPower = 0.0;
    private static Double quarterPower = 0.25;
    private static Double oneThirdPower = 0.34;
    private static Double halfPower = 0.5;
    private static Double threeQuartPower = 0.75;
    private static Double fullPower = 1.0;
    private static int sliderPosition = 0;

    private static double RClawOpen = -1.0;
    private static double RClawClosed = 0.315;
    private static double LClawOpen = 0.6;
    private static double LClawClosed = 0.40;
    private static double clawTiltUp = 0.55;
    private static double clawTiltDown = 1.0;
    private static double dropSample = 1.0;
    private static double clawStart = 0.85;
    private static int sampleArmUp = 330;
    private static int sampleArmDown = 700;
    private static double fingerStart = 0.0;
    private static double fingerUp = 0.6;
    private static double fingerTouch = 0.5;

    private static double anglePositionDown = 0.0;  // based on each servo need to find value when servo is changed
    private static double anglePositionUp = -1.0;   // based on each servo need to find value when servo is changed
    private static double hangLiftRobot = 270.0; // need to figure out


    //Low and High Rung
    private static double highRung = 500;


    // Global Variables to  Game Specific Information
    AutoMode      autoMode = AutoMode.AUTO_MODE_NOT_SELECTED;         // store automous mode selected
    ParkLocation  parkLocation = ParkLocation.PARK_LOC_NOT_SELECTED;  // store park location selected

    /*******************************************************************************************/
    /*

    /*******************************************************************************************/
    /* Function: SelectAutoMode                                                                */
    /* Returns: Selected mode                                                                  */
    /*                                                                                         */
    /* This function is use to select the automous code to be executed for this match          */
    /* Game pad 1 is used and the following buttons are used for selection:                    */
    /*      a - Samples                                                                        */
    /*      b - Specimen                                                                       */
    /*******************************************************************************************/

    private AutoMode SelectAutoMode()
    {
        AutoMode autoMode = AutoMode.AUTO_MODE_NOT_SELECTED;      // Local variable to store selected automous mode

        /*******************************************/
        /* Display automous mode not selected yet  */
        /*******************************************/
        telemetry.addData("AutoMode","Not Selected");
        telemetry.addData("AutoMode","Select a - Sample in bucket");
        telemetry.addData("AutoMode","Select b - Hang Specimen");
        telemetry.update();

        /*****************************************/
        /* Loop until automous mode is selected  */
        /*****************************************/
        while (!isStopRequested() && autoMode == AutoMode.AUTO_MODE_NOT_SELECTED)  {

            if (gamepad1.a) {  // Red side away from audience/Blue side toward audience
                autoMode = AutoMode.AUTO_MODE_SAMPLE;
            }

            if (gamepad1.b) {  // Red side toward audience/Blue side away from audience
                autoMode = AutoMode.AUTO_MODE_SPECIMEN;
            }

            idle();
        }

        /************************************/
        /* Display selected automous mode   */
        /************************************/
        // mode.setValue(autoMode.toString());
        telemetry.addData("Automous Mode", autoMode.toString());
        telemetry.update();

        // Wait for the user to release the button
        while (!isStopRequested() && (gamepad1.a || gamepad1.b || gamepad1.x || gamepad1.y)) {
            idle();
        }

        return autoMode;

    } // end SelectAutoMode


    /*******************************************************************************************/
    /* Function: SelectParkLoc                                                                 */
    /* Returns: Selected ParkLoc                                                               */
    /*                                                                                         */
    /* This function is use to select the automous code to be executed for this match          */
    /* Game pad 1 is used and the following buttons are used for selection:                    */
    /*      a - Observation Zone by wall                                                       */
    /*      b - Observation Zone away from the wall                                            */
    /*      x - L1 Ascent Right side                                                           */
    /*      y - L1 Ascent Left side                                                            */
    /*******************************************************************************************/

    private ParkLocation SelectParkLoc()
    {
        ParkLocation parkLoc = ParkLocation.PARK_LOC_NOT_SELECTED;      // Local variable to store selected automous mode

        /*******************************************/
        /* Display automous mode not selected yet  */
        /*******************************************/
        telemetry.addData("ParkLoc","Not Selected");
        telemetry.addData("ParkLoc","Select a - park Observation Zone by wall");
        telemetry.addData("ParkLoc","Select b - park Observation Zone away from wall");
        telemetry.addData("ParkLoc","Select x - park L1 Ascent Right side");
        telemetry.addData("ParkLoc","Select y - park L1 Ascent Left side");

        telemetry.update();

        /*****************************************/
        /* Loop until automous mode is selected  */
        /*****************************************/
        while (!isStopRequested() && parkLoc == ParkLocation.PARK_LOC_NOT_SELECTED)  {

            if (gamepad1.a) {  // Red side away from audience/Blue side toward audience
                parkLoc = ParkLocation.PARK_LOC_OBS_ZONE_WALL;
            }

            if (gamepad1.b) {  // Red side toward audience/Blue side away from audience
                parkLoc = ParkLocation.PARK_LOC_OBS_ZONE;
            }

            if (gamepad1.x) {
                parkLoc = ParkLocation.PARK_LOC_L1_ASCENT_RIGHT;
            }

            if (gamepad1.y){
                parkLoc = ParkLocation.PARK_LOC_L1_ASCENT_LEFT;
            }

            idle();
        }

        /************************************/
        /* Display selected park location mode   */
        /************************************/
        // mode.setValue(parkLoc.toString());
        telemetry.addData("Parking Location", parkLoc.toString());
        telemetry.update();

        // Wait for the user to release the button
        while (!isStopRequested() && (gamepad1.a || gamepad1.b || gamepad1.x || gamepad1.y)) {
            idle();
        }

        return parkLoc;

    } // end SelectParkLoc

    /*******************************************************************************************/
    /* Function: SelectDelayTime                                                               */
    /* Returns: Delay Time in milliseconds                                                     */
    /*                                                                                         */
    /* This function is use to select how long to delay the start of the automous code.        */
    /* Game pad 1 is used and the following controls are used for selection:                   */
    /*        left bumper - decrease delay time by 1000 milliseconds (1 second)                */
    /*        right bumper - increase delay time by 1000 milliseconds (1 second)               */
    /*        a button -    set selected time                                                  */
    /*                                                                                         */
    /*                    note: if no delay time is needed, just select the a button.     The  */
    /*                          default for the delay time is 0.                               */
    /*******************************************************************************************/
    private Integer SelectDelayTime()
    {
        Integer delayTimeMilliseconds = 0;    // Initialize delay to be 0 seconds

        // display delay time not set
        telemetry.addData("Delay","%d (Not Set)",delayTimeMilliseconds);
        telemetry.addData("Left Bumper -"," decrease delay time by (1 second)");
        telemetry.addData("Right bumper -", " increase delay time by (1 second)");
        telemetry.addData("A", "No delay time is needed");
        telemetry.update();

        NormalizedRGBA colors = color.getNormalizedColors();
        Color.colorToHSV(colors.toColor(), hsvValues);

        telemetry.addLine();
        telemetry.addData("Red", "%.3f", colors.red);
        telemetry.addData("Green", "%.3f", colors.green);
        telemetry.addData("Blue", "%.3f", colors.blue);

        telemetry.addLine();
        telemetry.addData("Hue","%.3f", hsvValues[0]);
        telemetry.addData("Saturation", "%.3f", hsvValues[1]);
        telemetry.addData("Value", "%.3f", hsvValues[2]);
        telemetry.addData("Alpha", "%.3f", colors.alpha);

        /*******************************************************************************/
        /* Select Delay time.                                                          */
        /*   - Select 'a' button without hitting bumpers if no delay needed            */
        /*   - Use Left Bumper to decrease delay time                                  */
        /*   - Use Right bumper to increase delay time                                 */
        /*                                                                             */
        /* Note:    After entering delay time, use "a" button to set selected time     */
        /*******************************************************************************/
        while (!isStopRequested() && gamepad1.a == false)
        {
            if (gamepad1.left_bumper)
            {
                delayTimeMilliseconds -= 1000;

                // ensure delay time does not go negetive
                if (delayTimeMilliseconds < 0)
                {
                    delayTimeMilliseconds = 0;
                }

                // Wait for the bumper to be released
                while (gamepad1.left_bumper)
                {
                    idle();
                }

                telemetry.addData("Delay","%d (decrease)",delayTimeMilliseconds);
                telemetry.update();
            }

            if (gamepad1.right_bumper)
            {
                delayTimeMilliseconds += 1000;

                // ensure delay time is not greater than 10 seconds
                if (delayTimeMilliseconds > 10000)
                {
                    delayTimeMilliseconds = 10000;
                }

                while (gamepad1.right_bumper)
                {
                    idle();
                }
                telemetry.addData("Delay","%d (increase)",delayTimeMilliseconds);
                telemetry.update();
            }
        }

        // Wait for user to release the a button
        while (!isStopRequested() && gamepad1.a)
        {
            idle();
        }

        /**************************************/
        /* Display selected delay time        */
        /**************************************/
        // delay.setValue("%d", delayTimeMilliseconds);
        telemetry.addData("Delay Time SET", delayTimeMilliseconds);
        telemetry.update();
        return delayTimeMilliseconds;       // returns selected delay time

    } // end SelectDelayTime

    /******************************/
    /* OpMode for automous code   */
    /******************************/
    @Override
    public void runOpMode() throws InterruptedException
    {

        double tgtPower = 0;

        /********************/
        /* Map All Motors   */
        /********************/
        FLMoto = hardwareMap.get(DcMotor.class, "FLMoto");
        FRMoto = hardwareMap.get(DcMotor.class, "FRMoto");
        BLMoto = hardwareMap.get(DcMotor.class, "BLMoto");
        BRMoto = hardwareMap.get(DcMotor.class, "BRMoto");

        //tapeSens = hardwareMap.get(ColorSensor.class, "bottomSens");


        stringMoto = hardwareMap.dcMotor.get("slider");
        climbMoto = hardwareMap.dcMotor.get("climb");
        hangMoto = hardwareMap.dcMotor.get("hang");

        VoltSens = hardwareMap.voltageSensor.get("Control Hub");

        /********************/
        /* Map All Servos   */
        /********************/

        LClaw = hardwareMap.servo.get("leftClaw");
        RClaw = hardwareMap.servo.get("rightClaw");
        clawTilt = hardwareMap.servo.get("angle");
        FINGER = hardwareMap.servo.get("finger");

        //map sensor
        color = hardwareMap.get(NormalizedColorSensor.class, "Csensor");


        /************************/
        /* Setup IMU parameters */
        /************************/

        // Retrieve and initialize the IMU. The IMU should be attached to
        // IC2 port 0 on a Core Device Interface Module
        imu = hardwareMap.get(IMU.class, "imu");

        /* The next two lines define Hub orientation.
         * The Default Orientation (shown) is when a hub is mounted horizontally with the printed logo pointing UP and the USB port pointing FORWARD.
         *
         * To Do:  EDIT these two lines to match YOUR mounting configuration.
         */
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;

        telemetry.addData("Mode","calibrating imu...." );
        telemetry.update();

        try {
            RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
            imu.initialize(new IMU.Parameters(orientationOnRobot));

            telemetry.addData("imu calib status", "calibrated" );
            telemetry.update();

            // initialize imu global variables after calibrating imu
            resetAngle();

        } catch (IllegalArgumentException e) {
            telemetry.addData("imu calib status", "failed - try again");
            telemetry.update();
        }


        // set direction of motors
        FLMoto.setDirection(DcMotorSimple.Direction.REVERSE);
        FRMoto.setDirection(DcMotorSimple.Direction.FORWARD);
        BLMoto.setDirection(DcMotorSimple.Direction.REVERSE);
        BRMoto.setDirection(DcMotorSimple.Direction.FORWARD);

        // set values for arm motor


        // set values for string motor
        stringMoto.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        stringMoto.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        stringMoto.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        stringMoto.setTargetPosition(0);

        hangMoto.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hangMoto.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hangMoto.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hangMoto.setTargetPosition(0);

        clawTilt.setPosition(clawStart);
        FINGER.setPosition(fingerStart);
        RClaw.setPosition(RClawClosed);   //drops sample
        LClaw.setPosition(LClawClosed);

        //BlinkinLEDs
        // blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkinLed");
        // blinkinLedDriver.setPattern(BasePattern);

        // telemetry.addData("Pattern: ", BasePattern.toString());
        // telemetry.update();

        // Create local variable to store selected automous mode
        // and amount of delay time
        Integer delayTimeMilliseconds = 0;      // variable to store how long to delay before starting automous

        telemetry.setAutoClear(false);
        telemetry.addData("Status", "Initializing");
        telemetry.update();

        /***********************************************************/
        /* Select Automous Mode, Parking Location and Delay time      */
        /***********************************************************/
        autoMode = SelectAutoMode();
        parkLocation = SelectParkLoc();
        delayTimeMilliseconds = SelectDelayTime();

        /**********************************************************/
        /* All required data entered.  Automous is initialized.     */
        /**********************************************************/
        telemetry.addData("Status", "Initialized");
        telemetry.addData("mode","waiting for start");
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        /**********************************/
        /* Wait for start of the match     */
        /**********************************/
        waitForStart();

        telemetry.clearAll();               // clear display messages

        telemetry.addData("Mode", "running");
        telemetry.update();

        resetEncoders();

        // Delay start if needed
        if (delayTimeMilliseconds > 0) {
            telemetry.addData("Status","Delaying");    // display delay status
            telemetry.update();

            sleep(delayTimeMilliseconds);                       // wait selected amount of time

            telemetry.addData("Status","Running");  // dispay delay over and automous code is running
            telemetry.update();

        }

        telemetry.clearAll();               // clear display messages

        // Determine which automous code to run
        switch (autoMode) {
            case AUTO_MODE_SAMPLE:
                Sample();
                break;

            case AUTO_MODE_SPECIMEN:
                Specimen();
                break;

            case AUTO_MODE_NOT_SELECTED:
                // This one should not happen if it does do nothing
                break;
        }
    }

    /***************************************************************************/
    /* Function: Sample                                                        */
    /* Returns: none                                                           */
    /* Uses Global Variables: parkLocation                                     */
    /*                                                                         */
    /* This function is for when the robot is in the red alliance and starts   */
    /* in the backstage location                                               */
    /*                                                                         */
    /* The robot will place the preloaded purple pixel on the spike mark that  */
    /* contains game element.  The robot will then move to the backstage area  */
    /* where it will place the yellow pixel on the backdrop and then park next */
    /* to the wall so the other team can have access to the backdrop and then  */
    /* park.                                                                   */
    /***************************************************************************/
    private void Sample() {
        // add code to place sample in high or low basket
        //drive to basket
        // driveForward (24.0, halfPower);   //drive forward from wall
        // rotate (110, halfPower);//align with basket
        // driveForward (22.0, halfPower);     //get closer to basket
        // sleep(100); //sleep
        // strafeRight (8.0, halfPower); //align arm with basket
        // sleep(200);


        strafeRight (10.0, halfPower);
        sleep(300);

        NormalizedRGBA colors = color.getNormalizedColors();

        while ((colors.red < 0.008) & (colors.blue < 0.008)){
            driveForward(0.5, quarterPower);

            colors = color.getNormalizedColors();
        }

        sleep(2000);

        stringMoto.setTargetPosition(1800); //extend arm
        stringMoto.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        stringMoto.setPower(1.0);
        sleep(1500);

        clawTilt.setPosition(clawTiltUp);  //tilt claw
        sleep(200);

        driveForward(4.0, quarterPower);
        sleep(500);

        RClaw.setPosition(RClawOpen);   //drops sample
        LClaw.setPosition(LClawOpen);
        sleep(200);
        //raises to basket


        driveBackward(12.0, halfPower);
        sleep(300);

        //reset

        clawTilt.setPosition(clawStart);
        stringMoto.setTargetPosition(0); //retract arm
        stringMoto.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        stringMoto.setPower(1.0);
        sleep(1000);


        sleep(300);
        sleep(99999);
        //this is where we stopped, everything before should work fine
        //vice versa everything after this is still iffy and needs to be tested/fixed
        //start of second sample
        rotate (-110, halfPower);

        //drive to samples
        strafeRight(2.5, halfPower);
        // clawTilt.setPosition(0.5);
        RClaw.setPosition(RClawOpen);   //drops sample
        LClaw.setPosition(LClawOpen);
        sleep(450);

        //driveForward(0.1, halfPower);
        //sleep(200);

        //pick up sample

        sleep(1500);

        RClaw.setPosition(RClawClosed);   //grabs sample
        LClaw.setPosition(LClawClosed);
        sleep(99999);



        sleep(500);


        //scoring second sample
        rotate(110, halfPower);
        sleep(200);

        while ((colors.red < 0.008) & (colors.blue < 0.008)){
            driveForward(0.5, quarterPower);

            colors = color.getNormalizedColors();
        }

        // strafeRight(5.0, halfPower);


        stringMoto.setTargetPosition(1800); //extend arm
        stringMoto.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        stringMoto.setPower(1.0);
        sleep(1500);


        driveForward (4.0, halfPower);
        sleep(300);

        clawTilt.setPosition(clawTiltUp);  //tilt claw
        sleep(300);

        RClaw.setPosition(RClawOpen);   //drops sample
        LClaw.setPosition(LClawOpen);
        sleep(400);


        //end of

        driveBackward (6.0, halfPower);
        sleep(100);



        clawTilt.setPosition(clawStart);
        stringMoto.setTargetPosition(0); //retract arm
        stringMoto.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        stringMoto.setPower(1.0);
        sleep(100);


        sleep(500);

        // Determine where to park based on selected parking during initialization
        switch (parkLocation) {
            case PARK_LOC_OBS_ZONE_WALL:


                break;

            case PARK_LOC_OBS_ZONE:


                break;

            case PARK_LOC_L1_ASCENT_RIGHT:

                break;

            case PARK_LOC_L1_ASCENT_LEFT:
                // driveBackward(14.0, halfPower);
                // FINGER.setPosition(fingerTouch);
                // sleep(1500);
                // rotate
                break;

        }

    }  // End of RedSample()



    /****************************************************************************/
    /* Function: Specimen                                                      */
    /* Returns: none                                                           */
    /* Uses Global Variables: parkLocation                                     */
    /*                                                                         */
    /* This function is for when the robot is in the red alliance and starts   */
    /* in the front (audience) location                                        */
    /*                                                                         */
    /* The robot will place the preloaded purple pixel on the spike mark that  */
    /* contains game element.  The robot will then move under the stagedoor to */
    /* the backstage area where it will place the yellow pixel on the backdrop */
    /* and then park away from the wall.                                       */
    /***************************************************************************/
    private void Specimen() {

        //   drive halfway to chamber and tilt claw in position;
        //picking up second specimen

        driveForward(4.0, halfPower);


        sleep(2000);

        //move linear slider up to high rung
        stringMoto.setTargetPosition(800);
        stringMoto.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        stringMoto.setPower(1.0);
        sleep(1000);

        clawTilt.setPosition(0.65);

        //   drive the rest of the way to chamber
        driveForward(18.0, halfPower);
        sleep(1000);

        //place specimen on rung
        clawTilt.setPosition(clawTiltDown);
        sleep(200);

        //retract arm
        stringMoto.setTargetPosition(0);
        stringMoto.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        stringMoto.setPower(1.0);
        sleep(500);

        driveBackward(7.0, halfPower);
        sleep(300);

        RClaw.setPosition(RClawOpen);
        LClaw.setPosition(LClawOpen);
        sleep(100);

        //driveBackward(15.0, halfPower); //uncomment later


        //Going to face 2nd specimen
        driveBackward(8.0, halfPower);
        sleep(1000);

        strafeRight(18.0, halfPower);
        sleep(600);

        rotate(-110, halfPower);
        sleep(500);

        strafeLeft(7.5, halfPower);
        sleep(300);

        //picking up second specimen
        clawTilt.setPosition(0.75);
        RClaw.setPosition(RClawOpen);
        LClaw.setPosition(LClawOpen);



        sleep(1500);

        //training
        sleep(500);

        RClaw.setPosition(RClawClosed);
        LClaw.setPosition(LClawClosed);
        sleep(100);

        //reset to prepare to score


        clawTilt.setPosition(0.65);
        sleep(200);

        rotate(110, halfPower);
        sleep(500);

        strafeLeft(28.5, halfPower);
        sleep(600);

        //going to score 2nd specimen


        sleep(200);

        stringMoto.setTargetPosition(900);
        stringMoto.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        stringMoto.setPower(1.0);
        sleep(800);

        driveForward(10.5, halfPower);
        sleep(500);

        clawTilt.setPosition(clawTiltDown);
        driveBackward(6.0, halfPower);
        RClaw.setPosition(RClawOpen);
        LClaw.setPosition(LClawOpen);
        sleep(1200);


        stringMoto.setTargetPosition(-300);
        stringMoto.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        stringMoto.setPower(1.0);
        sleep(500);
        // Determine where to park based on selected parking during initialization
        switch (parkLocation) {
            case PARK_LOC_OBS_ZONE_WALL:

                strafeRight(60.0, halfPower);

                sleep(99999);
                break;

            case PARK_LOC_OBS_ZONE:

                strafeRight(45.0, halfPower);
                driveBackward(10.0, halfPower);

                sleep(99999);
                break;

            case PARK_LOC_L1_ASCENT_RIGHT:



                break;

            case PARK_LOC_L1_ASCENT_LEFT:
                rotate (-110, halfPower);
                break;

        }

    } // End of RedSpecimen()


    //**************************************************************
    // Functions needed for driving auto
    //

    private void resetEncoders()
    {
        FRMoto.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BRMoto.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FLMoto.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BLMoto.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private void runUsingEncoders()
    {
        FRMoto.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BRMoto.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FLMoto.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BLMoto.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void setDrivePower(double FLpower,double FRpower, Double BLpower, Double BRpower)
    {
        FRMoto.setPower(FRpower);
        BRMoto.setPower(BRpower);
        FLMoto.setPower(FLpower);
        BLMoto.setPower(BLpower);
    }

    /************************************************************/
    /* Function: strafeRight                                    */
    /* Returns: nothing                                         */
    /*                                                          */
    /* This function is called to have the robot move sideways  */
    /* in a right direction                                     */
    /************************************************************/
    private void strafeRight(Double inches,Double power)
    {
        driveInches(inches*straifFactor, power, -power, -power, power);
    }

    /************************************************************/
    /* Function: strafeLeft                                     */
    /* Returns: nothing                                         */
    /*                                                          */
    /* This function is called to have the robot move sideways  */
    /* in a left  direction                                     */
    /************************************************************/
    private void strafeLeft(Double inches,Double power)
    {
        driveInches(inches*straifFactor, -power, power, power, -power);
    }

    /************************************************************/
    /* Function: driveForward                                   */
    /* Returns: nothing                                         */
    /*                                                          */
    /* This function is called to have the robot move forward.  */
    /* The robot speed is passed in and that value is used for  */
    /* all wheels.                                              */
    /************************************************************/
    private void driveForward(Double inches,Double power)
    {
        driveInches(inches, power, power, power, power);
    }

    /***************************************************************/
    /* Function: driveBackward                                     */
    /* Returns: nothing                                            */
    /*                                                             */
    /* This function is called to have the robot move in reverse.  */
    /* The robot speed is passed in and that value is used for     */
    /* all wheels.                                                 */
    /***************************************************************/
    private void driveBackward(Double inches,Double power)
    {
        driveInches(inches, -power, -power, -power, -power);
    }

    /************************************************************/
    /*                                                          */
    /* Function: driveInches                                    */
    /* Returns: nothing                                         */
    /*                                                          */
    /* This function is called to have the robot move straigh   */
    /* in a forward or reverse direction.                       */
    /*                                                          */
    /* Strafe Forward = negative front wheels, positive back    */
    /* wheels                                                   */
    /************************************************************/
    private void driveInches(Double inches,Double FLpower,Double FRpower, Double BLpower, Double BRpower)
    {
        Double counts = inches * countsPerInch;

        resetEncoders();
        runUsingEncoders();

        voltage = VoltSens.getVoltage();  // read current battery voltage

        double FLpowerCont = ((batteryConst*FLpower)/voltage);
        double FRpowerCont = ((batteryConst*FRpower)/voltage);
        double BLpowerCont = ((batteryConst*BLpower)/voltage);
        double BRpowerCont = ((batteryConst*BRpower)/voltage);

        setDrivePower(FLpower, FRpower, BLpower, BRpower);

        while (opModeIsActive() &&
                (Math.abs(FLMoto.getCurrentPosition()) + Math.abs(FRMoto.getCurrentPosition()) /2) < Math.abs(counts))
        {
            // Use gyro to drive in a straight line.
            correction = checkDirection();

            // telemetry.addData("1 imu heading", lastAngles.firstAngle);
            // telemetry.addData("2 global heading", globalAngle);
            // telemetry.addData("3 correction", correction);
            // telemetry.update();

            setDrivePower(FLpower-correction, FRpower+correction, BLpower-correction, BRpower+correction);
            idle();
        }

        setDrivePower(noPower,noPower,noPower,noPower);       // Stop all motors
    }

    //***************************************************************************
    // Functions for turning and checking robot angle for correction
    //

    /***************************************************/
    /* Resets the cumulative angle tracking to zero.   */
    /***************************************************/
    private void resetAngle()
    {
        imu.resetYaw();
        lastAngles = imu.getRobotYawPitchRollAngles();

        globalAngle = 0;
    }

    /************************************************************/
    /* Get current cumulative angle rotation from last reset.   */
    /* @return Angle in degrees. + = left, - = right.           */
    /************************************************************/
    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        // Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();

        double deltaAngle = orientation.getYaw(AngleUnit.DEGREES) - lastAngles.getYaw(AngleUnit.DEGREES);

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = orientation;

        return globalAngle;
    }

    /**
     * See if we are moving in a straight line and if not return a power correction value.
     * @return Power adjustment, + is adjust left - is adjust right.
     */
    private double checkDirection()
    {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .05;

        angle = getAngle();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
     * @param degrees Degrees to turn, + is left - is right
     */
    private void rotate(int degrees, double power)
    {
        double    leftPower, rightPower;

        // restart imu movement tracking.
        resetAngle();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).
        if (degrees < 0)
        {    // turn right.
            leftPower = power;
            rightPower = -power;
        }
        else if (degrees > 0)
        {    // turn left.
            leftPower = -power;
            rightPower = power;
        }
        else return;

        // set power to rotate.
        setDrivePower(leftPower,rightPower,leftPower,rightPower);

        // rotate until turn is completed.
        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0) {}

            while (opModeIsActive() && getAngle() > degrees) {}
        }
        else    // left turn.
        {
            while (opModeIsActive() && getAngle() < degrees) {}
        }

        // turn the motors off.
        setDrivePower(0.0,0.0,0.0,0.0);

        // wait for rotation to stop.
        sleep(500);

        // reset angle tracking on new heading.
        resetAngle();
    }

}
