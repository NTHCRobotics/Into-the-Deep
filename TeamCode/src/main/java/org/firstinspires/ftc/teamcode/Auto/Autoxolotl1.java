
package org.firstinspires.ftc.teamcode.Auto;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.acmerobotics.dashboard.config.Config;

@Autonomous(name="Autoxolotl23", group="Omarxolotl")

@Config
public class Autoxolotl1 extends LinearOpMode
{
    //Setting Variables
    // Extras
    private int[] wheelTicks = {0, 0, 0, 0}; // FL = 0, FR = 1, BL = 2, BR = 3
    private final ElapsedTime runtime = new ElapsedTime();  // Timer, I just copy pasted, don't ask questions
    private double speedMod;
    private int[] viperSlideTargets = {0, 1600, 2500, 3245};

    private  int[] sprocketTargets  = {0, 245 , 760 , 970};

    private int viperlevel ;


    //Motors
    private DcMotorEx wheelFL; // Front left wheel
    private DcMotorEx wheelFR; // Front right wheel
    private DcMotorEx wheelBL; // Back left wheel
    private DcMotorEx wheelBR; // Back right wheel
    private DcMotorEx viper; //Vertical lift mechanism
    private DcMotorEx rocket; // Motor for rotate the Vertical lift

    //Servos
    private Servo claw; // Opening and closing of the claw
    private Servo rotateClaw; // Rotates the claw up and down





    @Override // Init?? I think?
    public void runOpMode() throws InterruptedException {
         /*
        Hardware maps everything
        Hardware map assigns the variables a physical port, you must set the ports to the String in quotes by configuring the control hub
         */

        //Motors, mounts variables to hardware ports.
        wheelFL = hardwareMap.get(DcMotorEx.class, "wheelFL");
        wheelFR = hardwareMap.get(DcMotorEx.class, "wheelFR");
        wheelBL = hardwareMap.get(DcMotorEx.class, "wheelBL");
        wheelBR = hardwareMap.get(DcMotorEx.class, "wheelBR");


        viper = hardwareMap.get(DcMotorEx.class, "viper");
        rocket = hardwareMap.get(DcMotorEx.class, "rocket");


        //------------SERVOS////
        claw = hardwareMap.get(Servo.class, "claw");
        rotateClaw = hardwareMap.get(Servo.class, "rotateClaw");

        //Wheels
        wheelFL.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        wheelFR.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        wheelBL.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        wheelBR.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        wheelFL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        wheelFR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        wheelBL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        wheelBR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        wheelFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wheelFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wheelBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wheelBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        wheelFL.setTargetPositionTolerance(20);
        wheelFR.setTargetPositionTolerance(20);
        wheelBL.setTargetPositionTolerance(20);
        wheelBR.setTargetPositionTolerance(20);


        wheelFL.setTargetPosition(20);
        wheelFR.setTargetPosition(20);
        wheelBL.setTargetPosition(20);
        wheelBR.setTargetPosition(20);

        wheelFL.setDirection(DcMotorSimple.Direction.REVERSE);
        wheelFR.setDirection(DcMotorSimple.Direction.REVERSE);
        wheelBL.setDirection(DcMotorSimple.Direction.REVERSE);
        wheelBR.setDirection(DcMotorSimple.Direction.REVERSE);



        wheelFL.setVelocity(2000);
        wheelFR.setVelocity(2000);
        wheelBL.setVelocity(2000);
        wheelBR.setVelocity(2000);





        // Lock Wheels
        wheelFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wheelFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wheelBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wheelBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);




        //No idea, copy pasted
        viper.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        viper.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        viper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        viper.setTargetPositionTolerance(50);
        viper.setTargetPosition(50);
        viper.setDirection(DcMotorSimple.Direction.REVERSE);
        viper.setVelocity(2000);
        viper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Sprocket Stuff
        rocket.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rocket.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rocket.setDirection(DcMotorSimple.Direction.FORWARD);
        rocket.setTargetPosition(0);
        rocket.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rocket.setVelocity(600);


        // No idea, copy pasted
        wheelFL.setDirection(DcMotorSimple.Direction.FORWARD);//REVERSE
        wheelFR.setDirection(DcMotorSimple.Direction.REVERSE);//FORWARD
        wheelBL.setDirection(DcMotorSimple.Direction.FORWARD);//FORWARD
        wheelBR.setDirection(DcMotorSimple.Direction.REVERSE);//REVERSE

        waitForStart(); // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();
        waitForStart(); // Wait for the game to start (driver presses PLAY)

        /*
        // AUTON STARTS PRETTY MUCH HERE
        // This moves forward, then bla bla bla...
         */

//        claw.setPosition(0.65);
        moveByJoystick(1.4, 0.2, 0.46, 0);
        rotate(0.8, 0.86);
          rotateClaw.setPosition(0.57);
          moveSprocket(3);
       moveViper(2);



    }

    /*
    // FUNCTIONS DEFINED HERE
     */

    public void moveByJoystick(double seconds, double leftX, double leftY, double rightX) // Moves the robot by simulating a joystick
    {
        // Just check out https://gm0.org/en/latest/docs/software/tutorials/mecanum-drive.html for explanation
        double x = -leftX;
        double y = -leftY;
        double rotation = -rightX;
        double FL = (y + x + rotation);
        double FR = (y - x - rotation);
        double BL = (y - x + rotation);
        double BR = (y + x - rotation);


        int newFL = (int) Math.round(FL * 2000);
        int newFR = (int) Math.round(FR * 2000);
        int newBL = (int) Math.round(BL * 2000);
        int newBR = (int) Math.round(BR * 2000);

        wheelFL.setTargetPosition(newFL);
        wheelFR.setTargetPosition(newFR);
        wheelFR.setTargetPosition(newBL);
        wheelFR.setTargetPosition(newBR);

        telemetry.addData("New Wheel FR", newFR);
        telemetry.update();

        wheelFL.setPower(FL);
        wheelFR.setPower(FR);
        wheelBL.setPower(BL);
        wheelBR.setPower(BR);

//        wheelFL.setPower(0.8);
//        wheelFR.setPower(0.8);
//        wheelBL.setPower(0.8);
//        wheelBR.setPower(0.8);



        while (wheelFL.isBusy() || wheelFR.isBusy() || wheelBL.isBusy() || wheelBR.isBusy())
        {
            // Wait till finished
        }
        stopAndLock();
    }

    public void positionViper(int viperTarget)
    {

        viper.setTargetPosition(viperSlideTargets[viperTarget]);
        viperWait();

    }

    public void positionSprocket(int sproketTarget){
        rocket.setTargetPosition(sprocketTargets[sproketTarget]);
        sprocketWait();
    }




    public void wait(double seconds) // Waits the amount of seconds specified
    {
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < seconds))
        {
            // Nothing?
        }
        runtime.reset();


    }

    public void viperWait() // Waits the amount of seconds specified
    {
        runtime.reset();
        while (viper.isBusy())
        {
            // Nothing?
        }
        runtime.reset();


    }

    public void sprocketWait() // Waits the amount of seconds specified
    {
        runtime.reset();
        while (rocket.isBusy())
        {
            // Nothing?
        }
        runtime.reset();


    }









    public void moveForward(double seconds, double power)
    {
        moveByJoystick(seconds, 0, power, 0);
    }

    public void rotate(double seconds, double power)
    {
        moveByJoystick(seconds, 0, 0, power);
    }

    public void strafe(double seconds, double power)
    {
        moveByJoystick(seconds, power, 0, 0);
    }

    public void moveViper(int viperTarget)
    {
        positionViper(viperTarget);
    }



    public void moveSprocket(int  sproketTarget){
        positionSprocket(sproketTarget);
    }

    public void stopAndLock() // Stops and gives breaks to all wheels
    {
        // Stops all wheels
        wheelFL.setPower(0);
        wheelFR.setPower(0);
        wheelBL.setPower(0);
        wheelBR.setPower(0);

        // Makes all wheels brake
        wheelFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wheelFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wheelBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wheelBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void freeMotors() // Makes it so that wheels aren't brake locked
    {
        wheelFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        wheelFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        wheelBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        wheelBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }
}