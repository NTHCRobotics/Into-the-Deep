package org.firstinspires.ftc.teamcode.Teleop;


import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;



import java.util.Arrays;
@TeleOp(name="drivercontrol", group="Monkeys")
//@Disabled  This way it will run on the robot
public class Drive_Control_Red extends OpMode {

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

    //Servos
    private Servo Claw2; // Second CLaw
    private Servo Claw; // Primary Claw

    //Sensors
    private ColorSensor colorSensor; // Color sensor for detecting objects/colors


    private double speedMod;
    private final boolean rumbleLevel = true;
    private double rotation = 0;
    final double TRIGGER_THRESHOLD = 0.75;
    private double previousRunTime;
    private double inputDelayInSeconds = .5;
    private int[] armLevelPosition = {0,1200,3270};
    private int[] SprocketLevelPosition = {0,200,750,1100};
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
        Claw = hardwareMap.get(Servo.class, "Claw");
        Claw2 = hardwareMap.get(Servo.class, "Claw2");

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

        //Sprocket Encoder
        Rocket.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Rocket.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Rocket.setTargetPosition(0);

        //Wheel Direction
        wheelFL.setDirection(DcMotorSimple.Direction.FORWARD);//REVERSE
        wheelFR.setDirection(DcMotorSimple.Direction.REVERSE);//FORWARD
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
        drivingControl();
        Verticallift();
       // DectectYellow();
        ClawGrip();
        Clawroation();
        RocketBoom();
        //  SampleShoot();
        //drive2();
        Speices();

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

    // Driving control for mecanum wheels
    public void drivingControl() {
        double r = Math.hypot(-gamepad1.right_stick_x, gamepad1.right_stick_y);  // Calculate magnitude of joystick input
        double robotAngle = Math.atan2(-gamepad1.right_stick_y, gamepad1.right_stick_x) - Math.PI / 4;  // Calculate robot's angle
        double rightX = -gamepad1.left_stick_y;  // Rotation from right stick
        rotation += 1 * rightX;

        // Calculate power for each wheel based on joystick inputs and rotation
        final double v1 = r * Math.cos(robotAngle) - rightX;
        final double v2 = r * Math.sin(robotAngle) + rightX;
        final double v3 = r * Math.sin(robotAngle) - rightX;
        final double v4 = r * Math.cos(robotAngle) + rightX;

        // Set power to each wheel, adjusting with speed modifier
        wheelFL.setPower(v1 * speedMod);
        wheelFR.setPower(-v2 * speedMod);
        wheelBL.setPower(v3 * speedMod);
        wheelBR.setPower(-v4 * speedMod);
    }
   public void drive2(){
    // Forward/backward movement controlled by left stick y-axis (negative because y is inverted)
    double drive = -gamepad1.left_stick_y;  // Forward and backward
    double strafe = gamepad1.left_stick_x;  // Strafing left and right
    double rotate = -gamepad1.right_stick_x;  // Rotation from right stick

    // Calculate power for each wheel based on forward/backward, strafing, and rotation
    final double v1 = drive + strafe + rotate;  // Front-left wheel
    final double v2 = drive - strafe - rotate;  // Front-right wheel
    final double v3 = drive - strafe + rotate;  // Back-left wheel
    final double v4 = drive + strafe - rotate;  // Back-right wheel

    // Set power to each wheel, adjusting with speed modifier
    wheelFL.setPower(v1 * speedMod);
    wheelFR.setPower(-v2 * speedMod);
    wheelBL.setPower(v3 * speedMod);
    wheelBR.setPower(-v4 * speedMod);
}


//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    // Method to control the vertical lift mechanism
    public void Verticallift() {
        if ((gamepad2.y) && (armLevel < armLevelPosition.length - 1) && (getRuntime() - previousRunTime >= inputDelayInSeconds)) {

            previousRunTime = getRuntime();
            armLevel++;
        }
       else if ((gamepad2.a) && (armLevel > 0) && (getRuntime() - previousRunTime >= inputDelayInSeconds)) {

            previousRunTime = getRuntime();
            armLevel--;


        }

//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
        //sets to driving level
        if (gamepad1.x || gamepad2.x) {
            armLevel = 1;
        }

        viper.setVelocity(1000);
        if (armLevel == 1) {
            viper.setVelocity(1000);
            //if statement to set speed only going down
        }

        if (getRuntime() - previousRunTime >= inputDelayInSeconds + .25) {

        }
        viper.setTargetPosition(armLevelPosition[armLevel]);
        viper.setTargetPositionTolerance(armLevelPosition[armLevel]);
    }

    // Method to control the rocket motor mechanism
    public void RocketBoom() {
        // Check if the dpad_up button on gamepad2 is pressed
        if ((gamepad2.dpad_up ) && (armLevel>1)){

            Rocket.setTargetPosition(1110);
            Rocket.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        if(gamepad2.dpad_left){
            Rocket.setTargetPosition(750);
            Rocket.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        if(gamepad2.dpad_right){
            Rocket.setTargetPosition(180);
            Rocket.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        // Check if the dpad_down button on gamepad2 is pressed
         if (gamepad2.dpad_down) {

           Rocket.setTargetPosition(0);
            Rocket.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }



        Rocket.setVelocity(750);
    }

    // Method to control the claw grip mechanism
    public void ClawGrip() {
        // Check if the left bumper on gamepad2 is pressed
        if (gamepad2.left_bumper ) {
            // Set the claw servo to move forward
            Claw.setPosition(1); // Opens the CLaw
        }
        // Check if the right bumper on gamepad2 is pressed
        else if ((gamepad2.right_bumper)) {
            // Set the claw servo to move backward
            Claw.setPosition(-0.5); // Close the Claw
        }
        // If neither bumper is pressed, set the claw to stationary position

    }

    // Method to control the claw rotation mechanism
    public void Clawroation() {
        // Check if the triangle button on gamepad1 is pressed
        if (gamepad1.y) {
            // Set the claw rotation to 50% position
            Claw2.setPosition(.50);
        }
        // Check if the square button on gamepad1 is pressed
        else if (gamepad1.a) {
            // Set the claw rotation to 0% position
            Claw2.setPosition(0);
        }
    }

    // Method to handle sample shooting based on color detection
    // public void SampleShoot() {
        // Check if the blue value is greater than the threshold
     //   if (blueValue > TARGET_BLUE_THRESHOLD) {
            // Set the claw to eject the blue sample
    //        Claw.setPosition(-1);
    //    }
        // Check if the blue value is less than the red threshold
     //   else if (blueValue < TARGET_RED_THRESHOLD) {
            // Keep the blue sample in the robot
     //       Claw.setPosition(0);
     //   }

        // Check if yellow is detected (red and green values are above thresholds and blue is below)
       // if (redValue > YELLOW_RED_THRESHOLD && greenValue > YELLOW_GREEN_THRESHOLD && blueValue < YELLOW_BLUE_THRESHOLD) {
            // Display that yellow is detected
           // telemetry.addData("Status", "Yellow Detected");
           // telemetry.update();
            // Keep the yellow sample in the robot
           // Claw.setPosition(0);
      //  } else {
            // Display that no yellow is detected
      //      telemetry.addData("Status", "No Yellow Detected");
      //      telemetry.update();
    //    }
    // }
    public void Speices () {


    }

}







/*
 * Code to run ONCE after the driver hits STOP
 */

/*
 * Code to run ONCE after the driver hits STOP
 */


//@Override