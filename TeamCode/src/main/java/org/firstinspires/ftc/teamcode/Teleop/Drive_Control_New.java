package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="drivercontroloneperson", group="Axoltl")
//@Disabled  This way it will run on the robot
public class Drive_Control_New extends OpMode {
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
    private DcMotorEx Viper; //Vertical lift mechanism
    private DcMotorEx Rocket; // Motor for rotate the Vertical lift
    private Servo HangRight;
    private Servo HangLeft;
    //Servos
    private Servo RotationalClaw; // Second CLaw
    private Servo Claw; // Primary Claw


    //Sensors
    private ColorSensor colorSensor; // Color sensor for detecting objects/colors

    // I don't know what I'm doing, but these two variables are for parallelCounter
    private double[] myPrevRuntime = {0, 0, 0, 0};
    private boolean[] hasDone = {false, false, false, false};
    private boolean[] hasPressed = {false, false, false, false};


    // Here is where my dilly dallying ends

    private double speedMod;
    private final boolean rumbleLevel = true;
    private double rotation = 0;
    final double TRIGGER_THRESHOLD = 0.75;
    private double previousRunTime;
    private double inputDelayInSeconds = .5;
    private int[] armLevelPosition = {0, 1600, 2500, 3250,};
    private int[] SprocketLevelPosition = {0, 200, 750, 1100};
    private int SprocketLevel;
    private int armLevel;
    private int test = 0;



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


        Viper = hardwareMap.get(DcMotorEx.class, "viper");
        Rocket = hardwareMap.get(DcMotorEx.class, "rocket");


        //------------SERVOS////
        Claw = hardwareMap.get(Servo.class, "claw");
        RotationalClaw = hardwareMap.get(Servo.class, "rotateClaw");
        //  HangRight = hardwareMap.get(Servo.class, "hangRight");
        //HangLeft = hardwareMap.get(Servo.class, "hangLeft");


        //Motor Encoders
        //Wheels


        wheelFL.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        wheelFR.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        wheelBL.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        wheelBR.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        // Viper Encoder
        Viper.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        Viper.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        Viper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Viper.setTargetPositionTolerance(25);
        Viper.setTargetPosition(0);
        Viper.setDirection(DcMotorSimple.Direction.FORWARD);
        Viper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        //Sprocket Encoder
        Rocket.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Rocket.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Rocket.setDirection(DcMotorSimple.Direction.FORWARD);
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
        Verticallift();
        ClawGrip();
        drive();
        RocketBoom();
        parallelRocket(50, 0);
        ClawPickUp();
        ClawScoreCommand();
        SystemPickUp();
        SystemRest();
        SystemScore();
        //  SampleShoot();


        // Display telemetry data for debugging and tracking
        telemetry.addData("Left Trigger Position", gamepad1.left_trigger);


        telemetry.addData("Status", "Run Time: " + runtime.toString());
        //Arm Data
        telemetry.addData("velocity", Viper.getVelocity());
        telemetry.addData("slidePosition", Viper.getCurrentPosition());
        telemetry.addData("is at target", !Viper.isBusy());
        telemetry.addData("Target Slide Position", armLevelPosition[armLevel]);
        telemetry.addData("Slide Position", Viper.getCurrentPosition());
        telemetry.addData("Velocity", Viper.getVelocity());
        telemetry.addData("is at target", !Viper.isBusy());
        telemetry.addData("Tolerance: ", Viper.getTargetPositionTolerance());
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
            Viper.setVelocity(2000);
            RotationalClaw.setPosition(.68);
        }

//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
        //sets to driving level



        if (getRuntime() - previousRunTime >= inputDelayInSeconds + .25) {

        }
        Viper.setTargetPosition(armLevelPosition[armLevel]);
        Viper.setTargetPositionTolerance(armLevelPosition[armLevel]);
    }

    // Method to control the rocket motor mechanism
    public void RocketBoom() {
        if (gamepad1.dpad_up) {
            // Scoring Postion
            Rocket.setTargetPosition(970);
            Rocket.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } else if (gamepad1.dpad_left) {
            // Hang Postion
            Rocket.setTargetPosition(760);
            Rocket.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } else if (gamepad1.dpad_right) {
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
        //else if (gamepad1.dpad_right) {
            //Rocket.setTargetPosition(210);
            //Rocket.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            //RotationalClaw.setPosition(0.6);

//        }


        Rocket.setVelocity(600);
    }

    // Method to control the claw grip mechanism
    public void ClawGrip() {
        // Check if the left bumper on gamepad2 is pressed

            if (gamepad2.right_trigger > 0 ) {
                Claw.setPosition(1);
            }
            // Score postion
            else if (gamepad2.left_trigger > 0) {
                Claw.setPosition(0.65); // Before: 55
            }

    }
    public  void ClawPickUp(){
        if ( gamepad1.right_bumper){
            RotationalClaw.setPosition(0);
            Claw.setPosition(0.65);

        }
    }
    public void ClawScoreCommand(){
        if (gamepad1.left_bumper){
            RotationalClaw.setPosition(.92);
            Claw.setPosition(0.65);
        }
    }

    public void SystemScore(){
        if (gamepad1.y){
            Rocket.setTargetPosition(970);
            RotationalClaw.setPosition(0.6);
            armLevel = 3;

        }
        Viper.setTargetPosition(armLevelPosition[armLevel]);
        Viper.setTargetPositionTolerance(armLevelPosition[armLevel]);

    }
    public void SystemRest(){
        if (gamepad1.a){
            Rocket.setTargetPosition(0);
            RotationalClaw.setPosition(0.6);
            armLevel = 0;

        }
        Viper.setTargetPosition(armLevelPosition[armLevel]);
        Viper.setTargetPositionTolerance(armLevelPosition[armLevel]);

    }
    public  void SystemPickUp(){
        if (gamepad1.x){
            Rocket.setTargetPosition(240);
            armLevel = 1;
            RotationalClaw.setPosition(0.6) ;

        }
        Viper.setTargetPosition(armLevelPosition[armLevel]);
        Viper.setTargetPositionTolerance(armLevelPosition[armLevel]);
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
        if (gamepad1.dpad_down || gamepad1.a) {
            hasPressed[id] = true;
        }


        if (hasPressed[id]) {
            if (!hasDone[id]) {
                myPrevRuntime[id] = Viper.getCurrentPosition();
                hasDone[id] = true;
                armLevel = 0;
            }


            if (Viper.getCurrentPosition() <= position) {
                Rocket.setTargetPosition(0);
                hasPressed[id] = false;
                myPrevRuntime[id] = 0;
                hasDone[id] = false;

            }

        }
    }

}