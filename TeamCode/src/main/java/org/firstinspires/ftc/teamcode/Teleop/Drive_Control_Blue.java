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
@TeleOp(name="drivercontrolblue", group="Monkeys")
//@Disabled  This way it will run on the robot
public class Drive_Control_Blue extends OpMode {
    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();  //timer

    /*
    Declare motors to type DcMotorEx

    Documentation:
    https://ftctechnh.github.io/ftc_app/doc/javadoc/com/qualcomm/robotcore/hardware/DcMotorEx.html
     */

    //Touch Sensors
    //private DigitalChannel intakeSensor;

    //Motors
    private Rev2mDistanceSensor sideLeftDistanceSensor;
    private Rev2mDistanceSensor sideRightDistanceSensor;
    private DcMotorEx wheelFL;
    private DcMotorEx wheelFR;
    private DcMotorEx wheelBL;
    private DcMotorEx wheelBR;
    private DcMotorEx viper;
    private DcMotorEx Rocket;

    //Servos
    private Servo Claw2;
    //private DcMotorEx Insertnamehere
    //private DcMotorEx Insertnamehere
    private Servo Claw;

    //Sensors
    private ColorSensor colorSensor;

    private double speedMod;
    private final boolean rumbleLevel = true;
    private double rotation = 0;
    final double TRIGGER_THRESHOLD = 0.75;
    private boolean isGrabbing = false;
    private double previousRunTime;
    private double inputDelayInSeconds = .5;
    private int[] armLevelPosition = {0, 1000, 2000};
    private int armLevel;
    // int redValue = colorSensor.red();
    // int blueValue = colorSensor.blue();
    // int greenValue = colorSensor.green();
    //  private static final int TARGET_RED_THRESHOLD = 100;  // Minimum red value for scoring color
    //   private static final int TARGET_BLUE_THRESHOLD = 100; // Minimum blue value for scoring color
    //  private static final int YELLOW_RED_THRESHOLD = 200;  // Minimum red value for yellow
    //   private static final int YELLOW_GREEN_THRESHOLD = 200; // Minimum green value for yellow
    //  private static final int YELLOW_BLUE_THRESHOLD = 100; // Maximum blue value for yellow


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialization Started");


        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        //Motors
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


        viper.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        viper.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        viper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        viper.setTargetPositionTolerance(50);
        viper.setTargetPosition(50);
        viper.setDirection(DcMotorSimple.Direction.FORWARD);

        wheelFL.setDirection(DcMotorSimple.Direction.REVERSE);//REVERSE
        wheelFR.setDirection(DcMotorSimple.Direction.FORWARD);//FORWARD
        wheelBL.setDirection(DcMotorSimple.Direction.FORWARD);//FORWARD
        wheelBR.setDirection(DcMotorSimple.Direction.REVERSE);//REVERSE

        //Sensors
        // colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialization Complete");

    }

    //-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {


    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {

        runtime.reset();
        previousRunTime = getRuntime();

    }

    //----------------------------------------------------------------------------------------------------------------------------------------------------
    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
//this will run the methods repeadtly
        precisionControl();
        drivingControl();
        Verticallift();
        //SampleRedshodot();
        RocketBoom();
        ClawGrip();
        Clawroation();

//________________________________________________________________________________________________________________________________________________________________________________________________________________-
        telemetry.addData("Left Trigger Position", gamepad1.left_trigger);


        //Arm Slide Data
        telemetry.addData("velocity", viper.getVelocity());
        telemetry.addData("slidePosition", viper.getCurrentPosition());
        telemetry.addData("is at target", !viper.isBusy());
        telemetry.addData("Target Slide Position", armLevelPosition[armLevel]);
        telemetry.addData("Slide Position", viper.getCurrentPosition());
        telemetry.addData("Velocity", viper.getVelocity());
        telemetry.addData("is at target", !viper.isBusy());
        telemetry.addData("Tolerance: ", viper.getTargetPositionTolerance());

        // Show the elapsed game time and power for each wheel.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "wheelFL (%.2f), front right (%.2f), back left (%.2f),  right (%.2f)", wheelFL, wheelFR, wheelBL, wheelBR);

//        telemetry.addData("range", String.format("%.3f cm", sideDistanceSensor.getDistance(DistanceUnit.CM)));
//        telemetry.addData("range edited", sideDistanceSensor.getDistance(DistanceUnit.CM));
        // Color Sensor Data
        // telemetry.addData("Red", redValue);
        //  telemetry.addData("Green", greenValue);
        //  telemetry.addData("Blue", blueValue);

        telemetry.update();
    }


    //_______________________________________________________________________________________________________________________________________________________
    public void precisionControl() {
        if (gamepad1.left_trigger > 0) {
            speedMod = .25;
            gamepad1.rumble(1, 1, 200);
//            gamepad2.rumble(1, 1, 200);
        } else if (gamepad1.right_trigger > 0) {

            speedMod = 0.5;
//            gamepad1.rumble(1, 1, 200);
            gamepad1.rumble(1, 1, 200);

        } else {
            speedMod = 1;
            gamepad1.stopRumble();
//            gamepad2.stopRumble();
            //youtube
        }
    }

    //____________________________________________________________________________________________________________________________________________________________________________
    public void drivingControl() {
        //gets controller input
        double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);

        //make calculations based upon the input
        double robotAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
        double rightX = -gamepad1.right_stick_x;
        rotation += 1 * rightX;
        final double v1 = r * Math.cos(robotAngle) - rightX;
        final double v2 = r * Math.sin(robotAngle) + rightX;
        final double v3 = r * Math.sin(robotAngle) - rightX;
        final double v4 = r * Math.cos(robotAngle) + rightX;

        //change the power for each wheel
        wheelFL.setPower(-v1 * speedMod);
        wheelFR.setPower(-v2 * speedMod);
        wheelBL.setPower(v3 * speedMod);
        wheelBR.setPower(v4 * speedMod);
    }

    //--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    public void Verticallift() {

        if ((gamepad1.dpad_up || gamepad2.dpad_up) && (armLevel < armLevelPosition.length - 1) && (getRuntime() - previousRunTime >= inputDelayInSeconds)) {

            previousRunTime = getRuntime();
            armLevel++;
        }
        else if ((gamepad1.dpad_down || gamepad2.dpad_down) && (armLevel > 0) && (getRuntime() - previousRunTime >= inputDelayInSeconds)) {

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


    //    //---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    public void RocketBoom() {

        if (gamepad2.y) {
            Rocket.setPower(1);

        } else if (gamepad2.x) {
            Rocket.setPower(-1);
        }
    }




    public void ClawGrip() {
        //Ccontinuous rotation servo
        if(gamepad2.left_bumper){
            Claw.setPosition(1.0);
        }else if(gamepad2.right_bumper){
            Claw.setPosition(-1);
        }else {
            Claw.setPosition(0);
        }


    }

    public void Clawroation() {
        // Positional Servo
        if (gamepad1.y) {
            Claw2.setPosition(.50);
        } else if (gamepad1.x) {
            Claw2.setPosition(0);
        }
    }

    public void SampleRedshoot(){
       // if(redValue > TARGET_RED_THRESHOLD){ // checks if the red value is greater than the threshold
      //      Claw.setPosition(-1); // sets the claw position to -1 if the red value is greater than the threshold
      //  }else if(redValue < TARGET_BLUE_THRESHOLD) {
            //Claw.setPosition(0);
    //    }
     //   if (redValue > YELLOW_RED_THRESHOLD && greenValue > YELLOW_GREEN_THRESHOLD && blueValue < YELLOW_BLUE_THRESHOLD) {
            // Yellow object detected
      //      telemetry.addData("Status", "Yellow Detected");
       //     telemetry.update();
      //      Claw.setPosition(0); // Keeps the yellow sample in the robot
      //  } else {
            // No yellow object detected
           // telemetry.addData("Status", "No Yellow Detected");
          //  telemetry.update();
        }

    }





/*
 * Code to run ONCE after the driver hits STOP
 */

/*
 * Code to run ONCE after the driver hits STOP
 */


//@Override