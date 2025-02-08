package org.firstinspires.ftc.teamcode.Teleop;

import com.acmerobotics.dashboard.canvas.Rotation;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="uno", group="Axoltl")
//@Disabled  This way it will run on the robot
public class monkey extends OpMode {
    // Declare OpMode members.
    // Timer for tracking the runtime of the robot's operation.
    private final ElapsedTime runtime = new ElapsedTime();  //timer

    /*
    Declare motors to type DcMotorEx

    Documentation:
    https://ftctechnh.github.io/ftc_app/doc/javadoc/com/qualcomm/robotcore/hardware/DcMotorEx.html
     */

    //Touch Sensors
    //private DigitalChannel intakeSensor;

    //Motors
    private DcMotorEx wheelFL; // Front left wheel
    private DcMotorEx wheelFR; // Front right wheel
    private DcMotorEx wheelBL; // Back left wheel
    private DcMotorEx wheelBR; // Back right wheel
    private DcMotorEx SwyftSlideJr;
    private DcMotorEx SwyftSlide;
    private DcMotorEx Rocket; // Motor for rotate the Vertical lift
    private Servo HangRight;
    private Servo HangLeft;
    //Servos
    private Servo RotationalClaw; // Second CLaw
    private Servo Claw; // Primary Claw
    private Servo rollClaw;


    //Sensors
    private ColorSensor colorSensor; // Color sensor for detecting objects/colors

    // I don't know what I'm doing, but these two variables are for parallelCounter
    private double[] myPrevRuntime = {0, 0, 0, 0};
    private boolean[] hasDone = {false, false, false, false};
    private boolean[] hasPressed = {false, false, false, false};


    // Here is where my dilly dallying ends

    private double speedMod = 1;
    private final boolean rumbleLevel = true;
    private double rotation = 0;
    final double TRIGGER_THRESHOLD = 0.75;
    private double previousRunTime;
    private double inputDelayInSeconds = .5;
    private int[] armLevelPosition = {0, 1300, 1900, 2820};
    private int[] SprocketLevelPosition = {0, 200, 750, 1100};
    private int SprocketLevel;
    private int armLevel;
    private int test = 0;
    private final int SWYFT_VELOCITY = 2000;
    private final double SCORING_ROTATION = 0.80;


    // wifi pass petAxoltol

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialization Started");


        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        //Motors, mounts variables to hardware ports.
        wheelFL = hardwareMap.get(DcMotorEx.class, "wheelFL");
        wheelFR = hardwareMap.get(DcMotorEx.class, "wheelFR");
        wheelBL = hardwareMap.get(DcMotorEx.class, "wheelBL");
        wheelBR = hardwareMap.get(DcMotorEx.class, "wheelBR");


        SwyftSlide = hardwareMap.get(DcMotorEx.class, "SwyftSlide");
        SwyftSlideJr = hardwareMap.get(DcMotorEx.class, "SwyftSlideJr");
        Rocket = hardwareMap.get(DcMotorEx.class, "rocket");


        //------------SERVOS////
        Claw = hardwareMap.get(Servo.class, "claw");
        RotationalClaw = hardwareMap.get(Servo.class, "rotateClaw");
        rollClaw = hardwareMap.get(Servo.class, "rollClaw");

        //  HangRight = hardwareMap.get(Servo.class, "hangRight");
        //HangLeft = hardwareMap.get(Servo.class, "hangLeft");


        //Motor Encoders
        //Wheels


        wheelFL.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        wheelFR.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        wheelBL.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        wheelBR.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        //SwyftSlide Encoder

        SwyftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //SwyftSlide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        SwyftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //SwyftSlide.setTargetPositionTolerance(0);
        SwyftSlide.setTargetPosition(0);
        SwyftSlide.setDirection(DcMotorSimple.Direction.FORWARD);
        SwyftSlide.setVelocity(SWYFT_VELOCITY);
       SwyftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // SwyftSlideJr Encoder

        SwyftSlideJr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // SwyftSlideJr.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        SwyftSlideJr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // SwyftSlideJr.setTargetPositionTolerance(0);
        SwyftSlideJr.setTargetPosition(0);
        SwyftSlideJr.setDirection(DcMotorSimple.Direction.REVERSE);
        SwyftSlideJr.setVelocity(SWYFT_VELOCITY);
        SwyftSlideJr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


      /*  //Sprocket Encoder
        Rocket.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Rocket.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Rocket.setDirection(DcMotorSimple.Direction.REVERSE);
        Rocket.setVelocity(1500);
        Rocket.setTargetPosition(0);*/

        //Wheel Direction
        wheelFL.setDirection(DcMotorSimple.Direction.FORWARD);//FORWARD
        wheelFR.setDirection(DcMotorSimple.Direction.REVERSE);//REVERSE
        wheelBL.setDirection(DcMotorSimple.Direction.FORWARD);//FORWARD
        wheelBR.setDirection(DcMotorSimple.Direction.REVERSE);//REVERSE

        //Sensors
        //colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialization Complete");


    }

    //-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        // Add any code here that needs to loop during the initialization phase
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        // Reset runtime when play is pressed
        runtime.reset();
        previousRunTime = getRuntime();

        //Sprocket Encoder
        Rocket.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Rocket.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Rocket.setDirection(DcMotorSimple.Direction.REVERSE);
        //
        // Rocket.setVelocity(1500);
        Rocket.setTargetPosition(0);
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        // These methods will continuously run in the teleop loop
       // precisionControl();
        Verticallift();
        ClawGrip();
        ClawPitch();
        rollClaw();
        drive();
        RocketBoom();
        parallelRocket(50, 0);
        parallelHang(560, 3);
        parallelScore(750, 1);
        parallelPickup(100, 2);
       // parallelGroundPickup(75,0);


        // Display telemetry data for debugging and tracking
        telemetry.addData("Left Trigger Position", gamepad1.left_trigger);


        telemetry.addData("Status", "Run Time: " + runtime.toString());
        //SwyftSlide Data
        telemetry.addData("velocity", SwyftSlide.getVelocity());
        telemetry.addData("is at target", !SwyftSlide.isBusy());
        telemetry.addData("Target Slide Position SwyftSlide", armLevelPosition[armLevel]);
        telemetry.addData("Slide Position", SwyftSlide.getCurrentPosition());

        //SwyftSlideJr Data

        telemetry.addData("velocity", SwyftSlideJr.getVelocity());
        telemetry.addData("is at target", !SwyftSlideJr.isBusy());
        telemetry.addData("Target Slide Position SwyftSlideJr", armLevelPosition[armLevel]);
        telemetry.addData("Slide Position", SwyftSlideJr.getCurrentPosition());
        //  telemetry.addData("Red", redValue);
        //   telemetry.addData("Green", greenValue);
        //   telemetry.addData("Blue", blueValue);
        telemetry.addData("Rocket Level", SprocketLevelPosition[SprocketLevel]);
        telemetry.addData("Rocket Position", Rocket.getCurrentPosition());
        telemetry.addData("Test", test);
        telemetry.addData("Has Pressed", hasPressed);
        telemetry.addData("Has Done", hasDone);
        telemetry.addData("Prev Runtime", myPrevRuntime);


        telemetry.update();
    }


    // Adjust speed for precision control based on trigger inputs
//    public void precisionControl() {
//        if (gamepad1.left_trigger > 0) {
//            if (speedMod == 0.7) {
//                speedMod = 1;
//                gamepad1.rumble(0, 0, 0);
//            } else
//                speedMod = 0.7;
//            gamepad1.rumble(0.5, 0.5, 200);
//        }
//        if (gamepad1.right_trigger > 0) {
//            if (speedMod == 0.3) {
//                speedMod = 1;
//                gamepad1.rumble(0, 0, 0);
//            } else
//                speedMod = 0.3;
//            gamepad1.rumble(1, 1, 200);
//        }
//    }

    public void precisionControl() {
        speedMod = 1 - gamepad1.left_trigger * .8;
        gamepad1.rumble(gamepad1.left_trigger, gamepad1.left_trigger, 200); // Rumble feedback for precision mode

    }

    public void drive() {
        double x = -gamepad1.left_stick_x;
        double y = gamepad1.left_stick_y;
        double rotation = -gamepad1.right_stick_x;
        double FL = (y + x + rotation) * speedMod;
        double FR = (y - x - rotation) * speedMod;
        double BL = (y - x + rotation) * speedMod;
        double BR = (y + x - rotation) * speedMod;

        wheelFL.setPower(FL);
        wheelFR.setPower(FR);
        wheelBL.setPower(BL);
        wheelBR.setPower(BR);



    }


    //--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    // Method to control the vertical lift mechanism
    public void Verticallift() {
        if (gamepad2.b) {
            armLevel = 2;

            RotationalClaw.setPosition(.68);
        }
        if (gamepad2.y) {
            armLevel = 3;


        }
        if (gamepad2.a){
            armLevel = 0;
        }



//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
        //sets to driving level


        if (getRuntime() - previousRunTime >= inputDelayInSeconds + .25) {

        }
        SwyftSlide.setTargetPosition(armLevelPosition[armLevel]);
        SwyftSlide.setTargetPositionTolerance(armLevelPosition[armLevel]);

        SwyftSlideJr.setTargetPosition(armLevelPosition[armLevel]);
        SwyftSlideJr.setTargetPositionTolerance(armLevelPosition[armLevel]);
    }

    // Method to control the rocket motor mechanism
    public void RocketBoom() {
        if (gamepad1.dpad_up) {
            if (Rocket.getTargetPosition() == 700) {
                RotationalClaw.setPosition(SCORING_ROTATION);
            } else {
                // Scoring Postion
                // Rocket.setTargetPosition(700);
                Rocket.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
        } else if (gamepad2.dpad_left) {
            // Hang Postion
            Rocket.setTargetPosition(560);
            Rocket.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } else if (gamepad1.dpad_right ) {
            // Pick Up postion
            Rocket.setTargetPosition(55);
            Rocket.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            RotationalClaw.setPosition(0.6);
        }
// Check if the dpad_down button on gamepad2 is pressed
//        else if (gamepad1.dpad_down) {
//            // Rest Postion
//            Rocket.setTargetPosition(0);
//            Rocket.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//            RotationalClaw.setPosition(0.68);

//        }
        //else if (gamepad1.dpad_right) {
        //Rocket.setTargetPosition(210);
        //Rocket.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //RotationalClaw.setPosition(0.6);

//        }


    }

    // Method to control the claw grip mechanism

    public void ClawGrip() {
//        // Check if the left bumper on gamepad2 is pressed
        if (gamepad1.left_trigger > 0) {
            Claw.setPosition(0.5); //open
        }
//        // Score postion
        else if (gamepad1.right_trigger > 0) {
            Claw.setPosition(0); // close
        }
    }

    public void ClawPitch() {

        if (gamepad1.left_bumper) {
            RotationalClaw.setPosition(SCORING_ROTATION);
        }
        // Score postion
        if (gamepad1.right_bumper) {
            RotationalClaw.setPosition(0.1);
        }
        if (gamepad1.options) {

            RotationalClaw.setPosition(0.15
            );

        }

    }

    public void rollClaw(){
        if(gamepad1.dpad_up){
            rollClaw.setPosition(0); //fliped pitch
        }else if (gamepad1.dpad_down){
            rollClaw.setPosition(1);
        }
        else if (gamepad1.dpad_right
        ){


            rollClaw.setPosition(0.33);
        }

    }






    public void SecondHang() {
        //Going Down
        if (gamepad2.dpad_up) {
            HangRight.setPosition(0.7); // Correct Postion
            HangLeft.setPosition(0.57);
        }
        // Going Up
        else if (gamepad2.dpad_down) {
            HangRight.setPosition(0); // Correct Postion
            HangLeft.setPosition(1);

        }

    }


    public void baseParallel(double seconds, int id) // Uses previous runtime and runtime to make a parallel timer, variables are in array bc we need multiple
    // ID is incrememntal for arrays, seconds is how much waiting
    {
        if (true) // Put the controls that you want to have pressed for timer to start here
        {
            hasPressed[id] = true;
        }

        if (hasPressed[id]) {
            if (!hasDone[id]) {
                myPrevRuntime[id] = getRuntime();
                hasDone[id] = true;
            }


            if (getRuntime() >= seconds + myPrevRuntime[id]) {
                test = 1;
                hasPressed[id] = false;
                myPrevRuntime[id] = 0;
                hasDone[id] = false;

            }

        }



    }

    public void parallelRocket(int position, int id) // Parallell waiting for sprocket
    {
        Rocket.setVelocity(800);
        if (gamepad1.a) {
            hasPressed[id] = true;


        }


        if (hasPressed[id]) {
            if (!hasDone[id]) {
                myPrevRuntime[id] = SwyftSlide.getCurrentPosition();
                myPrevRuntime[id] = SwyftSlideJr.getCurrentPosition();

                hasDone[id] = true;
                armLevel = 0;
            }


            if (SwyftSlide.getCurrentPosition() <= position && SwyftSlideJr.getCurrentPosition() <= position) {
                Rocket.setTargetPosition(0);

                hasPressed[id] = false;
                myPrevRuntime[id] = 0;
                hasDone[id] = false;
                RotationalClaw.setPosition(0.30);
            }

        }


    }

    public void parallelScore(int position, int id) // Parallell waiting for sprocket

    {
        Rocket.setVelocity(1200);
        if (gamepad1.y) {
            hasPressed[id] = true;
            RotationalClaw.setPosition(SCORING_ROTATION);
            rollClaw.setPosition(1);
        }


        if (hasPressed[id]) {
            if (!hasDone[id]) {
                myPrevRuntime[id] = Rocket.getCurrentPosition();
                Rocket.setTargetPosition(750);
                hasDone[id] = true;
            }


            if (Rocket.getCurrentPosition() >= position) {
                armLevel = 3;

                hasPressed[id] = false;
                myPrevRuntime[id] = 0;
                hasDone[id] = false;


            }


        }
    }

    public void parallelPickup(int position, int id) { // Parallell waiting for sprocket
        {
            Rocket.setVelocity(1200);
            if (gamepad1.b) {
                hasPressed[id] = true;
                RotationalClaw.setPosition(0);
            }


            if (hasPressed[id]) {
                if (!hasDone[id]) {
                    myPrevRuntime[id] = Rocket.getCurrentPosition();
                    Rocket.setTargetPosition(100);
                    hasDone[id] = true;
                }


                if (Rocket.getCurrentPosition() >= position) {
                    armLevel = 1;
                    hasPressed[id] = false;
                    myPrevRuntime[id] = 0;
                    hasDone[id] = false;


                }


            }
        }
    }
    public void parallelGroundPickup(int position, int id2) { // Parallell waiting for sprocket
        {
            Rocket.setVelocity(1200);
            if (gamepad1.x) {
                hasPressed[id2] = true;
                RotationalClaw.setPosition(0);
            }


            if (hasPressed[id2]) {
                if (!hasDone[id2]) {
                    myPrevRuntime[id2] = Rocket.getCurrentPosition();
                    Rocket.setTargetPosition(75);
                    hasDone[id2] = true;
                }


                if (Rocket.getCurrentPosition() >= position) {
                    armLevel = 1;
                    hasPressed[id2] = false;
                    myPrevRuntime[id2] = 0;
                    hasDone[id2] = false;


                }


            }
        }
    }
    public void parallelHang(int position, int id) { // Parallell waiting for sprocket
        {
            Rocket.setVelocity(1200);
            if (gamepad2.dpad_left) {
                hasPressed[id] = true;
                RotationalClaw.setPosition(0.5);
            }


            if (hasPressed[id]) {
                if (!hasDone[id]) {
                    myPrevRuntime[id] = Rocket.getCurrentPosition();
                    Rocket.setTargetPosition(560);
                    hasDone[id] = true;
                }


                if (Rocket.getCurrentPosition() >= position) {
                    armLevel = 2;
                    hasPressed[id] = false;
                    myPrevRuntime[id] = 0;
                    hasDone[id] = false;


                }


            }
        }
    }

}



