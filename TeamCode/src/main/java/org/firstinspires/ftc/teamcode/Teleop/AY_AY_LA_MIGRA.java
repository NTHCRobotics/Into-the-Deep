/*
Wheels
    FR: 3 ex
    BR: 2 ex

 */

package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="AY_AY_LA_MIGRA", group="Axoltl")
//@Disabled  This way it will run on the robot
public class AY_AY_LA_MIGRA extends OpMode {
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
    private DcMotorEx SwyftSlide; //Vertical lift mechanism
    private DcMotorEx SwyftSlideJr; //Vertical lift mechanism
    private DcMotorEx Rocket; // Motor for rotate the Vertical lift
    private Servo HangRight;
    private Servo HangLeft;
    //Servos
    private Servo RotationalClaw; // Second CLaw
    private Servo Claw;
    private Servo rollClaw;// Primary Claw


    //Sensors
    private ColorSensor colorSensor; // Color sensor for detecting objects/colors

    // I don't know what I'm doing, but these two variables are for parallelCounter
    private double[] myPrevRuntime = {0, 0, 0, 0, 0, 0, 0};
    private boolean[] hasDone = {false, false, false, false, false, false, false};
    private boolean[] hasPressed = {false, false, false, false, false, false, false};


    // Here is where my dilly dallying ends

    private double speedMod;
    private final boolean rumbleLevel = true;
    private double rotation = 0;
    final double TRIGGER_THRESHOLD = 0.75;
    private double previousRunTime;
    private double inputDelayInSeconds = .5;
    private int[] armLevelPosition = {0, 1300, 1900, 2800,};
    private int[] SprocketLevelPosition = {0, 200, 750, 1100};
    private int SprocketLevel;
    private int armLevel;
    private int test = 0;


    private double clawRuntime = 0.0;
    private int clawNumber = 0;
    private boolean clawPressedL = false;
    private boolean clawPressedR = false;
    private double clawWait = 0; //I don't even know at this point.


    //private int blueValue = colorSensor.blue();
    // private int redValue = colorSensor.red();
    // private int greenValue = colorSensor.green();
    //  private static final int YELLOW_RED_THRESHOLD = 200;  // Minimum red value for yellow
    // private static final int YELLOW_GREEN_THRESHOLD = 200; // Minimum green value for yellow
    // private static final int YELLOW_BLUE_THRESHOLD = 100; // Maximum blue value for yellow
    // private static final int TARGET_RED_THRESHOLD = 100;  // Minimum red value for scoring color
    //  private static final int TARGET_BLUE_THRESHOLD = 100; // Minimum blue value for scoring color

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
        Rocket.setVelocity(1200);
        Rocket.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);





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

        // SwyftSlide Encoder
        SwyftSlide.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
//        SwyftSlide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        SwyftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        SwyftSlide.setTargetPositionTolerance(25);
        SwyftSlide.setTargetPosition(0);
        SwyftSlide.setDirection(DcMotorSimple.Direction.FORWARD);
        SwyftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        SwyftSlide.setVelocity(2000);


        SwyftSlideJr.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
//        SwyftSlideJr.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        SwyftSlideJr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        SwyftSlideJr.setTargetPositionTolerance(25);
        SwyftSlideJr.setTargetPosition(0);
        SwyftSlideJr.setDirection(DcMotorSimple.Direction.REVERSE);
        SwyftSlideJr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        SwyftSlideJr.setVelocity(2000);



        //Sprocket Encoder
        Rocket.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Rocket.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Rocket.setDirection(DcMotorSimple.Direction.REVERSE);
        Rocket.setTargetPosition(0);

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
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        // These methods will continuously run in the teleop loop
        precisionControl();
        //   SecondHang();
        Verticallift();
        // DectectYellow();
        clawGrip();
        clawRotation();
        clawRoll();
        claw(0.4, 0.13);
        drive();
        RocketBoom();
        parallelDDown(50, 0);
        parallelHang(560,3);
//        parallelScore(700, 1);
//        parallelPickup(100, 2);
//        parallelRetract(0, 3);
//        parallelExtend(0, 4);
//        parallelSubmersible(0, 5);
//        parallelSquareButton(0, 6);

        //  SampleShoot();


        // Display telemetry data for debugging and tracking
        telemetry.addData("Right Trigger Position", gamepad1.right_trigger);


        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Status", "Claw Run Time: " + clawRuntime);
        telemetry.addData("Claw Stuff", "Num: " + clawNumber + " L: " + clawPressedL + " R: " + clawPressedR);

        //Arm Data
        telemetry.addData("velocity", SwyftSlide.getVelocity());
        telemetry.addData("slidePosition", SwyftSlide.getCurrentPosition());
        telemetry.addData("is at target", !SwyftSlide.isBusy());
        telemetry.addData("Target Slide Position", armLevelPosition[armLevel]);
        telemetry.addData("Slide Position", SwyftSlide.getCurrentPosition());
        telemetry.addData("Velocity", SwyftSlide.getVelocity());
        telemetry.addData("is at target", !SwyftSlide.isBusy());
        telemetry.addData("Tolerance: ", SwyftSlide.getTargetPositionTolerance());
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
        if (gamepad1.b) {
            armLevel = 2;

            RotationalClaw.setPosition(.68);
        }
        if (gamepad1.y) {
            armLevel = 3;


        }
        if (gamepad1.a){
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
            // Scoring Postion
            Rocket.setTargetPosition(970);
            Rocket.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        else if (gamepad1.dpad_left) {
            // Hang Postion
            Rocket.setTargetPosition(730);
            Rocket.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } else if (gamepad1.share) {
            // Pick Up postion
            Rocket.setTargetPosition(245);
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
        else if (gamepad1.dpad_right) {
            Rocket.setTargetPosition(210);
            Rocket.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            RotationalClaw.setPosition(0.6);

        }


        Rocket.setVelocity(600);
    }
    public void claw(double cushion, double waitCushion) {  //HORRIBLE code, PLEASE make better. Y'all I tried
        if (clawWait + waitCushion < getRuntime()) { //This is for... stuff?


            if (gamepad1.right_trigger > 0) {
                if (gamepad1.left_bumper) {
                    clawRuntime = getRuntime();
                    if (clawNumber == 1) {
                        clawPressedL = true;
                    }
                    clawNumber = 1;
                    clawWait = getRuntime();


                }
                if (gamepad1.right_bumper) {
                    //clawDown
                    clawRuntime = getRuntime();
                    if (clawNumber == 2) {
                        clawPressedR = true;
                    }
                    clawNumber = 2;
                    clawWait = getRuntime();


                }
            } else {
                if (gamepad1.left_bumper) {
//                    clawRuntime = getRuntime();
//                    if (clawNumber == 3) {
//                        clawPressedL = true;
//                    }
//                    clawNumber = 3;
//                    clawWait = getRuntime();
//
                    Claw.setPosition(0.5); // remove this and add commented lines below for the other controlls


                }
                if (gamepad1.right_bumper) {
                    //close
//                    clawRuntime = getRuntime();
//                    if (clawNumber == 4) {
//                        clawPressedR = true;
//                    }
//                    clawNumber = 4;
//                    clawWait = getRuntime();
//
                    Claw.setPosition(0); // remove this and add commented lines below for the other controlls


                }
            }


            if (clawNumber == 1) //yo I don't even know what I'm doing
            {
                if (clawRuntime + cushion < getRuntime()) {
                    RotationalClaw.setPosition(0.5);
                    clawPressedL = false;
                    clawNumber = 0;
                } else if (clawPressedL == true) {
                    rollClaw.setPosition(0);
                    clawPressedL = false;
                    clawNumber = 0;

                }
            }

            if (clawNumber == 2) //yo I don't even know what I'm doing
            {
                if (clawRuntime + cushion < getRuntime()) {
                    RotationalClaw.setPosition(0.15);
                    clawPressedR = false;
                    clawNumber = 0;
                } else if (clawPressedR == true) {
                    rollClaw.setPosition(1);
                    clawPressedR = false;
                    clawNumber = 0;

                }
            }

//            if (clawNumber == 3) //yo I don't even know what I'm doing
//            {
//                if (clawRuntime + cushion < getRuntime()) {
//                    Claw.setPosition(0.5);
//                    clawPressedL = false;
//                    clawNumber = 0;
//                } else if (clawPressedL == true) {
//                    rollClaw.setPosition(0);
//                    clawPressedL = false;
//                    clawNumber = 0;
//
//                }
//            }
//
//            if (clawNumber == 4) //yo I don't even know what I'm doing
//            {
//                if (clawRuntime + cushion < getRuntime()) {
//                    Claw.setPosition(0);
//                    clawPressedR = false;
//                    clawNumber = 0;
//                } else if (clawPressedR == true) {
//                    rollClaw.setPosition(0.33);
//                    clawPressedR = false;
//                    clawNumber = 0;
//
//                }
//            }

        }
    }


    public void SecondHang() {
        //Going Down
        if (gamepad1.dpad_up) {
            HangRight.setPosition(0.7); // Correct Postion
            HangLeft.setPosition(0.57);
        }
        // Going Up
        else if (gamepad1.dpad_down) {
            HangRight.setPosition(0); // Correct Postion
            HangLeft.setPosition(1);

        }

    }

    public void clawGrip() {
        // Check if the right bumper on gamepad2 is pressed
        if (gamepad1.right_trigger < 0.4) {
            if (gamepad1.left_bumper) {
                Claw.setPosition(1);
            }
            // Score postion
            else if (gamepad1.right_bumper) {
                Claw.setPosition(0); // Before: 55
            }

            test = 1;
        }

        else
        {
            if(gamepad1.share)
            {
                Claw.setPosition(1);
            }
            else if (gamepad1.start)
            {
                Claw.setPosition(0);
            }

            test = 0;
        }
    }

    public void clawRotation() {
        if (gamepad1.right_trigger != 0)
        {
            if (gamepad1.right_bumper) {

                RotationalClaw.setPosition(0.8);
            }
            // Score postion
            if (gamepad1.left_bumper) {
                RotationalClaw.setPosition(0.50);
            }
        }

    }

    public void clawRoll(){

        if (gamepad1.right_trigger != 0)
        {
            if (gamepad1.right_bumper) {

                rollClaw.setPosition(0.5);
            }
            // Score postion
            if (gamepad1.left_bumper) {
                rollClaw.setPosition(1);
            }
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

       public void parallelDDown(int position, int id) // Parallell waiting for sprocket
    {
        if (gamepad1.dpad_down || gamepad1.a) {
            hasPressed[id] = true;
        }


        if (hasPressed[id]) {
            if (!hasDone[id]) {
                myPrevRuntime[id] = SwyftSlide.getCurrentPosition();
                hasDone[id] = true;
                if (Rocket.getCurrentPosition() < 10)
                {
                    Rocket.setTargetPosition(10);
                }
                armLevel = 0;
            }


            if (SwyftSlide.getCurrentPosition() <= position) {
                Rocket.setTargetPosition(0);
                hasPressed[id] = false;
                myPrevRuntime[id] = 0;
                hasDone[id] = false;

            }

        }
    }

    public void parallelScore(int position, int id) // Parallell waiting for sprocket
    {
        if (gamepad1.dpad_up) {
            hasPressed[id] = true;
            RotationalClaw.setPosition(0.15);
            Rocket.setTargetPosition(700);

        }


        if (hasPressed[id]) {
            if (!hasDone[id]) {
                myPrevRuntime[id] = Rocket.getCurrentPosition();
                Rocket.setTargetPosition(700);
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

    public void parallelExtend(double seconds, int id) // Uses previous runtime and runtime to make a parallel timer, variables are in array bc we need multiple
    // ID is incrememntal for arrays, seconds is how much waiting
    {
        if (gamepad1.y) // Put the controls that you want to have pressed for timer to start here
        {
            hasPressed[id] = true;
        }

        if (hasPressed[id]) {
            if (!hasDone[id]) {
                myPrevRuntime[id] = getRuntime();
                hasDone[id] = true;
                Rocket.setTargetPosition(13);
            }


            if (Rocket.getCurrentPosition() >= 13) {
                test = 1;
                hasPressed[id] = false;
                myPrevRuntime[id] = 0;
                hasDone[id] = false;
                armLevel = 3;

            }

        }



    }

    public void parallelSquareButton(double seconds, int id) // Uses previous runtime and runtime to make a parallel timer, variables are in array bc we need multiple
    // ID is incrememntal for arrays, seconds is how much waiting
    {
        if (gamepad1.x) // Put the controls that you want to have pressed for timer to start here
        {
            hasPressed[id] = true;
        }

        if (hasPressed[id]) {
            if (!hasDone[id]) {
                myPrevRuntime[id] = getRuntime();
                hasDone[id] = true;
                if (Rocket.getCurrentPosition() < 10)
                {
                    Rocket.setTargetPosition(10);
                }
            }


            if (Rocket.getCurrentPosition() >= 10) {
                test = 1;
                hasPressed[id] = false;
                myPrevRuntime[id] = 0;
                hasDone[id] = false;
                armLevel = 1;

            }

        }



    }



    public void parallelPickup(int position, int id) { // Parallell waiting for sprocket
        {
            if (gamepad1.dpad_right || gamepad1.b) {
                hasPressed[id] = true;
                RotationalClaw.setPosition(0);
            }


            if (hasPressed[id]) {
                if (!hasDone[id]) {
                    myPrevRuntime[id] = Rocket.getCurrentPosition();
                    Rocket.setTargetPosition(70);
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
    public void parallelHang(int position, int id) { // Parallell waiting for sprocket
        {
            if (gamepad1.dpad_left) {
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

    public void parallelRetract(double seconds, int id) // Uses previous runtime and runtime to make a parallel timer, variables are in array bc we need multiple
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

    public void parallelSubmersible(double seconds, int id) // Uses previous runtime and runtime to make a parallel timer, variables are in array bc we need multiple
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

    public void parallelHang(double seconds, int id) // Uses previous runtime and runtime to make a parallel timer, variables are in array bc we need multiple
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



    public void parallelOtherExtend(double seconds, int id) // Uses previous runtime and runtime to make a parallel timer, variables are in array bc we need multiple
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






}