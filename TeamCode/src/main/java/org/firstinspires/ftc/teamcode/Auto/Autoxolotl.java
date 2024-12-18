
package org.firstinspires.ftc.teamcode.Auto;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.acmerobotics.dashboard.config.Config;

@Autonomous(name="Autoxolotl", group="Omarxolotl")

@Config
public class Autoxolotl extends LinearOpMode
{
    //Setting Variables
    // Extras
    private final ElapsedTime runtime = new ElapsedTime();  // Timer, I just copy pasted, don't ask questions
    private double speedMod;
    private int[] viperSlideTargets = {0, 1600,2500,3245};

    private  int[] sprocketTargets  = {0,245 , 760 , 970};


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
    public void runOpMode() throws InterruptedException
    {
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

        moveForward(0.5, -0.5);
         rotate(0.2,0.5);
        moveSprocket(2,3);
         moveViper(2,2);


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

        wheelFL.setPower(FL);
        wheelFR.setPower(FR);
        wheelBL.setPower(BL);
        wheelBR.setPower(BR);

        wait(seconds);

        stopAndLock();
    }

    public void Positionviper(double seconds, int viperTarget)
    {

        viper.setTargetPosition(viperSlideTargets[viperTarget]);
        viperwait(seconds);

    }

    public void PostionSprocket(double seconds, int sproketTarget){
        rocket.setTargetPosition(sprocketTargets[sproketTarget]);
        sprocketwait(seconds);
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

    public void viperwait(double seconds) // Waits the amount of seconds specified
    {
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < seconds) && viper.isBusy() )
        {
            // Nothing?
        }
        runtime.reset();


    }

    public void sprocketwait(double seconds) // Waits the amount of seconds specified
    {
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < seconds) || rocket.isBusy())
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

    public void moveViper(double seconds, int viperTarget)
    {
        Positionviper(seconds , viperTarget);
    }



    public void moveSprocket(double seconds, int  sproketTarget){
        PostionSprocket(seconds, sproketTarget);
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
