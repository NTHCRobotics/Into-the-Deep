package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="drivercontrolaxoltl10", group="Axoltl")
//@Disabled  This way it will run on the robot
public class Drive_Control extends OpMode {
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
    private DcMotorEx viper; //Vertical lift mechanism
    private DcMotorEx Rocket; // Motor for rotate the Vertical lift
    private Servo HangRight;
    private Servo HangLeft;
    //Servos
    private Servo RotationalClaw; // Second CLaw
    private Servo Claw; // Primary Claw


    //Sensors
    private ColorSensor colorSensor; // Color sensor for detecting objects/colors


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
    //private int blueValue = colorSensor.blue();
    // private int redValue = colorSensor.red();
    // private int greenValue = colorSensor.green();
    //  private static final int YELLOW_RED_THRESHOLD = 200;  // Minimum red value for yellow
    // private static final int YELLOW_GREEN_THRESHOLD = 200; // Minimum green value for yellow
    // private static final int YELLOW_BLUE_THRESHOLD = 100; // Maximum blue value for yellow
    // private static final int TARGET_RED_THRESHOLD = 100;  // Minimum red value for scoring color
    //  private static final int TARGET_BLUE_THRESHOLD = 100; // Minimum blue value for scoring color

    // wifi pass Petaxolotl

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


        viper = hardwareMap.get(DcMotorEx.class, "viper");
        Rocket = hardwareMap.get(DcMotorEx.class, "rocket");


        //------------SERVOS////
        Claw = hardwareMap.get(Servo.class, "claw");
        RotationalClaw = hardwareMap.get(Servo.class, "rotateClaw");
        HangRight = hardwareMap.get(Servo.class, "hangRight");
        HangLeft = hardwareMap.get(Servo.class, "hangLeft");


        //Motor Encoders
        //Wheels


        wheelFL.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        wheelFR.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        wheelBL.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        wheelBR.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        // Viper Encoder
        viper.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        viper.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        viper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        viper.setTargetPositionTolerance(50);
        viper.setTargetPosition(50);
        viper.setDirection(DcMotorSimple.Direction.REVERSE);
        viper.setVelocity(7000);

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
        SecondHang();
        Verticallift();
        // DectectYellow();
        ClawGrip();
        drive();
        RocketBoom();
        ClawRotation();
        //  SampleShoot();


        // Display telemetry data for debugging and tracking
        telemetry.addData("Left Trigger Position", gamepad1.left_trigger);
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        //Arm Data
        telemetry.addData("velocity", viper.getVelocity());
        telemetry.addData("slidePosition", viper.getCurrentPosition());
        telemetry.addData("is at target", !viper.isBusy());
        telemetry.addData("Target Slide Position", armLevelPosition[armLevel]);
        telemetry.addData("Slide Position", viper.getCurrentPosition());
        telemetry.addData("Velocity", viper.getVelocity());
        telemetry.addData("is at target", !viper.isBusy());
        telemetry.addData("Tolerance: ", viper.getTargetPositionTolerance());
        //  telemetry.addData("Red", redValue);
        //   telemetry.addData("Green", greenValue);
        //   telemetry.addData("Blue", blueValue);
        telemetry.addData("Rocket Level", SprocketLevelPosition[SprocketLevel]);
        telemetry.addData("Rocket Position", Rocket.getCurrentPosition());
        telemetry.update();
    }


    // Adjust speed for precision control based on trigger inputs
    public void precisionControl() {
        if (gamepad1.left_trigger > 0) {
            speedMod = .25;
            gamepad1.rumble(1, 1, 200);  // Rumble feedback for precision mode
        } else if (gamepad1.right_trigger > 0) {
            speedMod = 0.5;
            gamepad1.rumble(1, 1, 200);  // Rumble feedback for medium speed mode
        } else {
            speedMod = 1;
            gamepad1.stopRumble();  // Stop rumble if neither trigger is pressed
        }
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
        if ((gamepad2.y) && (armLevel < armLevelPosition.length - 1) && (getRuntime() - previousRunTime >= inputDelayInSeconds)) {
            RotationalClaw.setPosition(.68);
            armLevel = 3;


        } else if ((gamepad2.a) && (armLevel > 0) && (getRuntime() - previousRunTime >= inputDelayInSeconds)) {

            armLevel = 0;
            RotationalClaw.setPosition(.68);




        } else if (gamepad2.b) {
            armLevel = 2;
            viper.setVelocity(10000);
            RotationalClaw.setPosition(.68);
        }

//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
        //sets to driving level
        if (gamepad2.x) {
            armLevel = 1;

        RotationalClaw.setPosition(.68);

        }




        if (getRuntime() - previousRunTime >= inputDelayInSeconds + .25) {

        }
        viper.setTargetPosition(armLevelPosition[armLevel]);
        viper.setTargetPositionTolerance(armLevelPosition[armLevel]);

    }

    // Method to control the rocket motor mechanism
    public void RocketBoom() {
        // Check if the dpad_up button on gamepad2 is pressed
        if (gamepad2.dpad_up) {
            // Scoring Postion
            Rocket.setTargetPosition(970);
            Rocket.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } else if (gamepad2.dpad_left) {
            // Hang Postion
            Rocket.setTargetPosition(760);
            Rocket.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } else if (gamepad2.share) {
            // Pick Up postion
            Rocket.setTargetPosition(245);
            Rocket.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            RotationalClaw.setPosition(0.6);
        }
// Check if the dpad_down button on gamepad2 is pressed
        else if (gamepad2.dpad_down) {
            // Rest Postion
            Rocket.setTargetPosition(0);
            Rocket.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            RotationalClaw.setPosition(0.68);

        }
        else if (gamepad2.dpad_right){
            Rocket.setTargetPosition(225);
            Rocket.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            RotationalClaw.setPosition(0.6);

        }


        Rocket.setVelocity(1200);
    }

    // Method to control the claw grip mechanism
    public void ClawGrip() {
        // Check if the left bumper on gamepad2 is pressed
        if (gamepad2.left_trigger > 0) {
            // Set the claw servo to move forward
            Claw.setPosition(1);// Opens the CLaw
        }
        // Check if the right bumper on gamepad2 is pressed
        else if (gamepad2.right_trigger > 0) {
            // Set the claw servo to move backward
            Claw.setPosition(0.65); // Close the Claw
        }
        // If neither bumper is pressed, set the claw to stationary position

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

    public void ClawRotation() {
        if (gamepad2.left_bumper) {
            RotationalClaw.setPosition(0.8);
        }
        // Score postion
        else if (gamepad2.right_bumper) {
            RotationalClaw.setPosition(0.43);
        }
    }

    // Method to control the claw rotation mechanism


}





/*
 * Code to run ONCE after the driver hits STOP
 */


/*
 * Code to run ONCE after the driver hits STOP
 */


//@Override