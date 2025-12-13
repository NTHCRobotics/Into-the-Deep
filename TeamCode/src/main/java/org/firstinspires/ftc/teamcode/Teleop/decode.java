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

@TeleOp(name="decode", group="Axoltl")
//@Disabled  This way it will run on the robot
public class decode extends OpMode {
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

    private DcMotorEx intakeF; // Front Intake
    private DcMotorEx intakeB; // Back Intake
    private DcMotorEx flyWheel;
    private DcMotorEx rotateHood;

    //Servos
    private Servo elevator; // Lifts Artifacts
    private Servo magazine; // Moves magazine

    //Sensors
    private ColorSensor colorSensor; // Color sensor for detecting objects/colors

    private double speedMod;
    private final boolean rumbleLevel = true;


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

        intakeF = hardwareMap.get(DcMotorEx.class, "intakeF");
        intakeB = hardwareMap.get(DcMotorEx.class, "intakeB");
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        rotateHood = hardwareMap.get(DcMotorEx.class, "rotateHood");


    //Servos
    private Servo elevator; // Lifts Artifacts
    private Servo magazine; // Moves magazine

    //Sensors
    private ColorSensor colorSensor; // Color sensor for detecting objects/colors





        //------------SERVOS////
        elevator = hardwareMap.get(Servo.class, "elevator");
        magazine = hardwareMap.get(Servo.class, "magazine");

        //Motor Encoders
        //Wheels


        wheelFL.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        wheelFR.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        wheelBL.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        wheelBR.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        //Wheel Direction
        wheelFL.setDirection(DcMotorSimple.Direction.FORWARD);//FORWARD
        wheelFR.setDirection(DcMotorSimple.Direction.REVERSE);//REVERSE
        wheelBL.setDirection(DcMotorSimple.Direction.FORWARD);//FORWARD
        wheelBR.setDirection(DcMotorSimple.Direction.REVERSE);//REVERSE

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
        drive();
        flywheel();
//        telemetry.addData("Prev Runtime", myPrevRuntime);
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


    public void intake()
    {
            if (gamepad1.right_trigger > 0)
            {
                    intakeF.setPower(1);
                    intakeB.setPower(1);
            }
      else 
      {
        intakeF.setPower(0);
        intakeB.setPower(0);
      }
    }

    public void flywheel()
  {
    if (gamepad1.left_bumper)
    {
      flywheel.setPower(1);
    }
      else
    {
        flywheel-setPower(0);
    }
      
  }

