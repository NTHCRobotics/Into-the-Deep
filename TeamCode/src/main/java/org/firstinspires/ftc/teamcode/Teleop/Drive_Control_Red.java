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
    private DcMotorEx Verticallift; //Vertical lift mechanism
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
    private int blueValue = colorSensor.blue();
    private int redValue = colorSensor.red();
    private int greenValue = colorSensor.green();
    private static final int YELLOW_RED_THRESHOLD = 200;  // Minimum red value for yellow
    private static final int YELLOW_GREEN_THRESHOLD = 200; // Minimum green value for yellow
    private static final int YELLOW_BLUE_THRESHOLD = 100; // Maximum blue value for yellow
    private static final int TARGET_RED_THRESHOLD = 100;  // Minimum red value for scoring color
    private static final int TARGET_BLUE_THRESHOLD = 100; // Minimum blue value for scoring color

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


        Verticallift = hardwareMap.get(DcMotorEx.class, "verticallift");
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


        Verticallift.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        Verticallift.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        Verticallift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Verticallift.setTargetPositionTolerance(50);
        Verticallift.setDirection(DcMotorSimple.Direction.REVERSE);

        wheelFL.setDirection(DcMotorSimple.Direction.REVERSE);//REVERSE
        wheelFR.setDirection(DcMotorSimple.Direction.FORWARD);//FORWARD
        wheelBL.setDirection(DcMotorSimple.Direction.FORWARD);//FORWARD
        wheelBR.setDirection(DcMotorSimple.Direction.REVERSE);//REVERSE

        //Sensors
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");

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
        DectectYellow();
        ClawGrip();
        Clawroation();
        RocketBoom();
        SampleShoot();
        Speices();

        // Display telemetry data for debugging and tracking
        telemetry.addData("Left Trigger Position", gamepad1.left_trigger);
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Red", redValue);
        telemetry.addData("Green", greenValue);
        telemetry.addData("Blue", blueValue);
        telemetry.update();
    }

    // Get the amount of red detected by the color sensor
    public int getAmountRed() {
        return colorSensor.red();
    }

    // Get the amount of blue detected by the color sensor
    public int getAmountBlue() {
        return colorSensor.blue();
    }

    // Detect yellow based on color thresholds
    public void DectectYellow() {
        if (redValue > YELLOW_RED_THRESHOLD && greenValue > YELLOW_GREEN_THRESHOLD && blueValue < YELLOW_BLUE_THRESHOLD) {
            telemetry.addData("Status", "Yellow Detected");
        } else {
            telemetry.addData("Status", "No Yellow Detected");
        }
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
        double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);  // Calculate magnitude of joystick input
        double robotAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;  // Calculate robot's angle
        double rightX = -gamepad1.right_stick_x;  // Rotation from right stick
        rotation += 1 * rightX;

        // Calculate power for each wheel based on joystick inputs and rotation
        final double v1 = r * Math.cos(robotAngle) - rightX;
        final double v2 = r * Math.sin(robotAngle) + rightX;
        final double v3 = r * Math.sin(robotAngle) - rightX;
        final double v4 = r * Math.cos(robotAngle) + rightX;

        // Set power to each wheel, adjusting with speed modifier
        wheelFL.setPower(-v1 * speedMod);
        wheelFR.setPower(-v2 * speedMod);
        wheelBL.setPower(v3 * speedMod);
        wheelBR.setPower(v4 * speedMod);
    }

    //--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    // Method to control the vertical lift mechanism
    public void Verticallift() {
        // Check if the circle button on gamepad2 is pressed
        if (gamepad2.circle) {
            // Set the target position of the vertical lift to 0 (lower position)
            Verticallift.setTargetPosition(0);
            // Set the motor to brake mode when not moving to hold its position
            Verticallift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // Check if the triangle button on gamepad2 is pressed
        if (gamepad2.triangle) {
            // Set the target position of the vertical lift to 2000 (intermediate position)
            Verticallift.setTargetPosition(2000);
            // Set the motor to float mode when not moving (allows free movement)
            Verticallift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }

        // Check if the square button on gamepad2 is pressed
        if (gamepad2.square) {
            // Set the target position of the vertical lift to 3000 (higher position)
            Verticallift.setTargetPosition(3000);
            Verticallift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }

        // Check if the X button on gamepad2 is pressed
        if (gamepad2.x) {
            // Set the target position of the vertical lift to 4000 (maximum position)
            Verticallift.setTargetPosition(4000);
            Verticallift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
    }

    // Method to control the rocket motor mechanism
    public void RocketBoom() {
        // Check if the dpad_up button on gamepad2 is pressed
        if (gamepad2.dpad_up) {
            // Set the rocket motor power to 1 (move forward)
            Rocket.setPower(1);
        }
        // Check if the dpad_down button on gamepad2 is pressed
        else if (gamepad2.dpad_down) {
            // Set the rocket motor power to -1 (move backward)
            Rocket.setPower(-1);
        }
        // If neither dpad_up nor dpad_down are pressed, stop the motor
        else {
            Rocket.setPower(0);
        }
    }

    // Method to control the claw grip mechanism
    public void ClawGrip() {
        // Check if the left bumper on gamepad2 is pressed
        if (gamepad2.left_bumper) {
            // Set the claw servo to move forward
            Claw.setPosition(1.0);
        }
        // Check if the right bumper on gamepad2 is pressed
        else if (gamepad2.right_bumper) {
            // Set the claw servo to move backward
            Claw.setPosition(-1);
        }
        // If neither bumper is pressed, set the claw to stationary position
        else {
            Claw.setPosition(0);
        }
    }

    // Method to control the claw rotation mechanism
    public void Clawroation() {
        // Check if the triangle button on gamepad1 is pressed
        if (gamepad1.triangle) {
            // Set the claw rotation to 50% position
            Claw2.setPosition(.50);
        }
        // Check if the square button on gamepad1 is pressed
        else if (gamepad1.square) {
            // Set the claw rotation to 0% position
            Claw2.setPosition(0);
        }
    }

    // Method to handle sample shooting based on color detection
    public void SampleShoot() {
        // Check if the blue value is greater than the threshold
        if (blueValue > TARGET_BLUE_THRESHOLD) {
            // Set the claw to eject the blue sample
            Claw.setPosition(-1);
        }
        // Check if the blue value is less than the red threshold
        else if (blueValue < TARGET_RED_THRESHOLD) {
            // Keep the blue sample in the robot
            Claw.setPosition(0);
        }

        // Check if yellow is detected (red and green values are above thresholds and blue is below)
        if (redValue > YELLOW_RED_THRESHOLD && greenValue > YELLOW_GREEN_THRESHOLD && blueValue < YELLOW_BLUE_THRESHOLD) {
            // Display that yellow is detected
            telemetry.addData("Status", "Yellow Detected");
            telemetry.update();
            // Keep the yellow sample in the robot
            Claw.setPosition(0);
        } else {
            // Display that no yellow is detected
            telemetry.addData("Status", "No Yellow Detected");
            telemetry.update();
        }
    }
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