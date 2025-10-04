package org.firstinspires.ftc.teamcode;

//color values red > 0.008 blue > 0.008
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Position;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import android.graphics.Color;
// Blinkin
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

@TeleOp(name = "TeleOpMain (Blocks to Java)", group = "Drive")
public class TeleOpMain extends LinearOpMode {


    private Servo angle;
    private Servo rightClaw;
    private Servo leftClaw;
    private Servo finger;
//    private boolean rightOpen = false;
//    private boolean leftOpen = false;

//    public boolean ltrig = false;
//    public boolean rtrig = false;


    private DcMotor climb;
    private DcMotor hang;

    private DcMotor slider;
    private double reduceSpeed = 1.0;

    private NormalizedColorSensor color;

    private static double powerConstant = .51;

    private static int sliderPosition = 0;
    private static int sliderPosPicPix = 1080;
    private static int outPosition = 0;

    private static double anglePosition = 0.9;
    private static double angleStartPosition = 0.9;
    private static int anglePositionDown = 600;
    private static int anglePositionUp = 500;
    //   private static int anglePosition = anglePositionDown;
    private static int climbPosition = 0;
    private static double rightClawOpen = -1.0;
    private static double rightClawClosed = 0.30;
    private static double leftClawOpen = 0.6;
    private static double leftClawClosed = 0.39;
    private static int climbUpPosition = 850;
    private static int climbDownPosition = 0;
    private static int clawHangPosUp = 650;
    private static int clawHangPosDown = 0;
    private static int hangLiftRobot = 270;
    private static int hangLiftDown = 0;
    private static double fingerstart = 2.0;


    // Blinkin
    private RevBlinkinLedDriver blinkinLedDriver;

    private RevBlinkinLedDriver.BlinkinPattern BasePattern = RevBlinkinLedDriver.BlinkinPattern.RED;
    private RevBlinkinLedDriver.BlinkinPattern StopPattern = RevBlinkinLedDriver.BlinkinPattern.WHITE;
    private RevBlinkinLedDriver.BlinkinPattern Level1Pattern = RevBlinkinLedDriver.BlinkinPattern.YELLOW;
    private RevBlinkinLedDriver.BlinkinPattern Level2Pattern = RevBlinkinLedDriver.BlinkinPattern.BLUE;
    private RevBlinkinLedDriver.BlinkinPattern Level3Pattern = RevBlinkinLedDriver.BlinkinPattern.GREEN;


    protected enum DisplayKind {
        MANUAL,
        AUTO
    }

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {

        double Voltage = 0;
        double wUP = .3;
        double Iin = 1;
        double Fly = 1;

        //****************************************//
        // Map all robot hardware                 //
        //****************************************//

        //BR and FR motors were mapped incorrectly on driver hub
        //Remapped FR back to "FRMoto", and BR back to "BRMoto" in the code to avoid confusion.
        //Drive Wheels
        DcMotor FLMoto = hardwareMap.dcMotor.get("FLMoto");
        DcMotor FRMoto = hardwareMap.dcMotor.get("FRMoto"); //Was BR
        DcMotor BLMoto = hardwareMap.dcMotor.get("BLMoto");
        DcMotor BRMoto = hardwareMap.dcMotor.get("BRMoto"); //Was FR


        //slider = hardwareMap.dcMotor.get("slider");
        //climb = hardwareMap.dcMotor.get("climb");
       // hang = hardwareMap.dcMotor.get("hang");

       // angle = hardwareMap.servo.get("angle");
       // rightClaw = hardwareMap.servo.get("rightClaw");
        //leftClaw = hardwareMap.servo.get("leftClaw");
        //finger = hardwareMap.servo.get("finger");

       // color = hardwareMap.get(NormalizedColorSensor.class, "Csensor");


        // Blinkin
        //blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkinLed");
        //blinkinLedDriver.setPattern(BasePattern);

        telemetry.addData("Pattern: ", BasePattern.toString());
        telemetry.update();

        float hsvValues[] = {0F, 0F, 0F};
        final float values[] = hsvValues;

        //*****************************************//
        // Put initialization blocks here.         //
        //*****************************************//

        //***************************************************//
        // Set direction of all motors                       //
        //***************************************************//

        FLMoto.setDirection(DcMotorSimple.Direction.REVERSE);
        FRMoto.setDirection(DcMotorSimple.Direction.FORWARD);
        BLMoto.setDirection(DcMotorSimple.Direction.REVERSE);
        BRMoto.setDirection(DcMotorSimple.Direction.FORWARD);





        //rightClaw.setPosition(rightClawClosed);
        //leftClaw.setPosition(leftClawClosed);
        //angle.setPosition(anglePosition);
        //finger.setPosition(fingerstart);

        // Wait for the start of TeleOp
        waitForStart();

        // Put run blocks here.

        //******************************************//
        // Run code while op mode is active         //
        //******************************************//
        while (opModeIsActive())
        {
            //   telemetry.addData("Status","opModeIsActive" );
            //   telemetry.addData("Arm Position:",armMoto.getCurrentPosition());
            //   telemetry.addData("Slider Extension:",slider.getCurrentPosition());
            //   telemetry.addData("Angle Position:", anglePosition );
            //   telemetry.addData("Hook Position:", climbPosition );
            //   telemetry.addData("Left Claw Position", leftOpen);
            //   telemetry.addData("Right Claw Position", rightOpen);
            //   telemetry.addData("Climb Position", climb.getCurrentPosition());
            //   telemetry.addData("Hang Position", hang.getCurrentPosition());
            //   telemetry.addData("tolerance", armMoto.getTargetPositionTolerance());
            //   telemetry.addDate("Extrapoint Position", extrapointPosition);

            // NormalizedRGBA colors = color.getNormalizedColors();
            // Color.colorToHSV(colors.toColor(), hsvValues);

            // telemetry.addLine();
            // telemetry.addData("Red", "%.3f", colors.red);
            // telemetry.addData("Green", "%.3f", colors.green);
            // telemetry.addData("Blue", "%.3f", colors.blue);

            // telemetry.addLine();
            // telemetry.addData("Hue","%.3f", hsvValues[0]);
            // telemetry.addData("Saturation", "%.3f", hsvValues[1]);
            // telemetry.addData("Value", "%.3f", hsvValues[2]);
            // telemetry.addData("Alpha", "%.3f", colors.alpha);
            telemetry.update();

            // Put loop blocks here
            // armMoto.setTargetPosition(sliderPosition);
            // slider.setTargetPosition(outPosition);
            // angle.setPosition(anglePosition);
            // climb.setTargetPosition(climbPosition);

            // bring all arm and slider
       /*if(gamepad2.x)
       {
          //outPosition = sliderPosPicPix;
          armMoto.setTargetPosition(anglePositionInit);
          armMoto.setMode(DcMotor.RunMode.RUN_TO_POSITION);
          armMoto.setPower(0.2);
          outPosition = 0;
          sliderPosition = 0;
       }*/
/*
            //brings arm to start position aswell as angle
            if (gamepad2.dpad_up)
            {
                //   sliderPosition = 0;
                //   slider.setTargetPosition(sliderPosition);
                //   slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                //   slider.setPower(0.2);
                //angle.setPosition(angleStartPosition);

                //   armMoto.setTargetPositionTolerance(10);

            }

            //Brings arm down to gram pixles
            if (gamepad2.dpad_left)
            {

            }

            //   if (gamepad2.right_stick_button == true)
            //   {
            //       angle.setPosition(0.6);
            //       armMoto.setTargetPosition(570);
            //       armMoto.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //       armMoto.setPower(0.2);
            //   }

            if (gamepad2.right_stick_button == true)
            {
                rightClaw.setPosition(rightClawOpen);
                leftClaw.setPosition(leftClawOpen);
                angle.setPosition(0.65);

            }

            if (gamepad2.left_stick_button == true)
            {
                slider.setTargetPosition(-1750);
                slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slider.setPower(1.0);
                sleep(1000);
                angle.setPosition(1.0);

            }
            //Brings arm down to gram pixles
            if (gamepad2.dpad_down)
            {
                leftClaw.setPosition(leftClawOpen);
                rightClaw.setPosition(rightClawOpen);

            }

            //Put Claw in Open Postion
            if (gamepad2.b)
            {
                leftClaw.setPosition(leftClawOpen);
                rightClaw.setPosition(rightClawOpen);
            }

            //Put Claw in Closed Position
            if (gamepad2.x)
            {
                leftClaw.setPosition(leftClawClosed);
                rightClaw.setPosition(rightClawClosed);
            }

            //Position Slider Lower Bucket Position
            if (gamepad2.y)
            {
                //Lean slightly Forward
                //Set Claw Position For Release
                angle.setPosition(0.5);
                sliderPosition = -4000;
                slider.setTargetPosition(sliderPosition);
                slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slider.setPower(0.9);

            }

            //Move Back to Initilization Position
            if (gamepad2.a)
            {
                angle.setPosition(0.6);
                slider.setTargetPosition(0);
                slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slider.setPower(1.0);

            }

            if(gamepad2.right_bumper)
            {
                sliderPosition = sliderPosition - 15;
                if (sliderPosition < -4500)
                {
                    sliderPosition = -4500;
                }
                slider.setTargetPosition(sliderPosition);
                slider.setPower(1.0);
                slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            if(gamepad2.left_bumper)
            {
                sliderPosition = sliderPosition + 15;
                if (sliderPosition > 0)
                {
                    sliderPosition = 0;
                }
                slider.setTargetPosition(sliderPosition);
                slider.setPower(2.0);
                slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                //angle.setPositon();
            }


            //claw angle
            if(gamepad2.left_trigger == 1)
            {
                anglePosition = anglePosition + 0.005;
                if (anglePosition > 1.0)
                {
                    anglePosition = 1.0;
                }
                angle.setPosition(anglePosition);
            }
            else if(gamepad2.right_trigger == 1)
            {
                anglePosition = anglePosition -0.005;
                if (anglePosition < 0)
                {
                    anglePosition = 0;
                }
                angle.setPosition(anglePosition);
            }


            if(gamepad2.dpad_right == true)
            {
                sliderPosition = sliderPosition - 1000;
                if (sliderPosition < -1000)
                {
                    sliderPosition = -1000;
                }
                slider.setTargetPosition(sliderPosition);
                slider.setPower(1.0);
                slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            //Movement

            //Speed Switch
            //   if(gamepad1.x == true)
            //   {
            //Slow Down Overall Speed
            //  reduceSpeed = 0.50;
            //   }
            //   else if(gamepad1.b == true)
            //   {
            //  Speed Up Overall Speed
            //  reduceSpeed = 1.5;
            //   }
*/
            //Strafe Left
            if (gamepad1.left_trigger > 0)
            {
                //Set Wheels into Strafe Position
                FRMoto.setPower(gamepad1.left_trigger * reduceSpeed);
                FLMoto.setPower(-gamepad1.left_trigger * reduceSpeed);
                BRMoto.setPower(-gamepad1.left_trigger * reduceSpeed);
                BLMoto.setPower(gamepad1.left_trigger * reduceSpeed);

                
            }

            //Strafe Right
            else if (gamepad1.right_trigger > 0)
            {
                //Set Wheels into Strafe Position
                FRMoto.setPower(-gamepad1.right_trigger);
                FLMoto.setPower(gamepad1.right_trigger);
                BRMoto.setPower(gamepad1.right_trigger);
                BLMoto.setPower(-gamepad1.right_trigger);

            }
            //If no strafe input Regular Drive Mechanics
            else
            {
                FRMoto.setPower(-gamepad1.right_stick_y);  //FL
                FLMoto.setPower(-gamepad1.left_stick_y);  //BR
                BRMoto.setPower(-gamepad1.right_stick_y); //BL
                BLMoto.setPower(-gamepad1.left_stick_y); //FR
            }
/*
            //climb claw - lift robot
            if(gamepad1.right_bumper == true)
            {
                climb.setTargetPosition(climbDownPosition);
                climb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                climb.setPower(1.0);

            }

            //climb claw - get into position
            if(gamepad1.left_bumper)
            {
                climb.setTargetPosition(climbUpPosition);
                climb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                climb.setPower(0.2);

            }

            //hang claw - lift arm into hanging position
            //  if(gamepad1.a == true)
            //  {
            //     hang.setTargetPosition(clawHangPosUp);
            //     hang.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //     hang.setPower(1.0);
            //  }

            //hang claw - lift arm into hanging position
            if(gamepad1.dpad_down == true)
            {
                hang.setTargetPosition(0);
                hang.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                hang.setPower(0.3);
            }

            //  if(gamepad1.dpad_right == true)
            //  {

            //  }

            if(gamepad1.dpad_up == true)
            {

            }

            if(gamepad1.dpad_left == true)
            {
                climb.setTargetPosition(climbUpPosition);
                climb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                climb.setPower(0.7);

                sleep(3500);

                hang.setTargetPosition(250);
                hang.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                hang.setPower(0.3);

                sleep(500);

                climb.setTargetPosition(climbDownPosition);
                climb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            if(gamepad1.dpad_right == true)
            {
                hang.setTargetPosition(-250);
                hang.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                hang.setPower(0.3);
            } */
        }
    }
}
