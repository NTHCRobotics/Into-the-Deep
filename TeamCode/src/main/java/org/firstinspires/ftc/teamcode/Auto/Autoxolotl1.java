
package org.firstinspires.ftc.teamcode.Auto;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.acmerobotics.dashboard.config.Config;

@Autonomous(name="Autoxolotl223", group="Omarxolotl")

@Config
public class Autoxolotl1 extends LinearOpMode
{
    //Setting Variables
    // Extras
    private int[] wheelTicks = {0, 0, 0, 0}; // FL = 0, FR = 1, BL = 2, BR = 3
    private final ElapsedTime runtime = new ElapsedTime();  // Timer, I just copy pasted, don't ask questions
    private double speedMod;
    private int[] viperSlideTargets = {0, 1600, 2500, 3245,5000};

    private  int[] sprocketTargets  = {0, 245 , 760 , 970};

    private int viperlevel ;


    //Motors
    private DcMotorEx wheelFL; // Front left wheel
    private DcMotorEx wheelFR; // Front right wheel
    private DcMotorEx wheelBL; // Back left wheel
    private DcMotorEx wheelBR; // Back right wheel
    private DcMotorEx viper; //Vertical lift mechanism
    private DcMotorEx rocket; // Motor for rotate the Vertical lift

    // Wheel Ticks
    private int newFL = 0;
    private int newFR = 0;
    private int newBL = 0;
    private int newBR = 0;

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

        wheelFL.setTargetPositionTolerance(100);
        wheelFR.setTargetPositionTolerance(100);
        wheelBL.setTargetPositionTolerance(100);
        wheelBR.setTargetPositionTolerance(100);


        wheelFL.setTargetPosition(0);
        wheelFR.setTargetPosition(0);
        wheelBL.setTargetPosition(0);
        wheelBR.setTargetPosition(0);

        wheelFL.setDirection(DcMotorSimple.Direction.FORWARD);
        wheelFR.setDirection(DcMotorSimple.Direction.FORWARD);
        wheelBL.setDirection(DcMotorSimple.Direction.FORWARD);
        wheelBR.setDirection(DcMotorSimple.Direction.FORWARD);



        wheelFL.setPower(0.6);
        wheelFR.setPower(0.6);
        wheelBL.setPower(0.6);
        wheelBR.setPower(0.6);






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
        viper.setDirection(DcMotorSimple.Direction.FORWARD);
        viper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        viper.setVelocity(5000);


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
     //   moveByJoystick(1.4, 0, 0, 0 , 1200);
//        moveForward(1.2,0.5,;



        // left x ==  -1 is left stafre
        // left x == 1 is right stafre
        // right x == 1 is clockwise
        // right x == -1 is counter clockwise

        telemetry.update();
        telemetry.update();
        // Pre Load Sample
        moveByJoystick(1,0,-1,0,700);
        moveByJoystick(0.5,-1,0,0,300);
        moveSprocket(3);
        moveByJoystick(0.5,0,0,-1,400);
        moveByJoystick(0.2,0,1,0,100);
        rotateClaw.setPosition(0.68);
        moveViper(3);
        rotateClaw.setPosition(0.43);
        claw.setPosition(1);
        rotateClaw.setPosition(.68);
        moveViper(0);
        moveSprocket(1);
        // First Sample
     moveByJoystick(0.5,0,0,-1,550);
     moveViper(2);
     rotateClaw.setPosition(0.7);
     claw.setPosition(0.65);
     moveViper(0);
     moveByJoystick(0.5,0,0,1,550);
     moveSprocket(3);
     rotateClaw.setPosition(0.68);
     moveViper(3);
     rotateClaw.setPosition(0.43);
     claw.setPosition(1);
     rotateClaw.setPosition(.68);
     moveViper(0);
     moveSprocket(1);
     //Second Sample
        moveByJoystick(0.4,0,0,-1,750);
        moveViper(2);
        rotateClaw.setPosition(0.7);
        claw.setPosition(0.65);
        moveViper(0);
        moveByJoystick(0.4,0,0,1,750);
        moveSprocket(3);
        rotateClaw.setPosition(0.68);
        moveViper(3);
        rotateClaw.setPosition(0.43);
        claw.setPosition(1);
        rotateClaw.setPosition(.68);
        moveViper(0);
        moveSprocket(1);

        //Thrid Sample
        moveByJoystick(1,0,0,-1 ,900);
        moveViper(2);
        rotateClaw.setPosition(0.7);
        claw.setPosition(0.65);
        moveViper(0);
        moveByJoystick(0.4,0,0,1,900);
        moveSprocket(3);
        rotateClaw.setPosition(0.68);
        moveViper(3);
        rotateClaw.setPosition(0.43);
        claw.setPosition(1);
        rotateClaw.setPosition(.68);
        moveViper(0);
        moveSprocket(1);
        // moveByJoystick(2, -0.5, 0, 0, -2000);

        //   rotate(0.8, 0.86 ,20) ;
        //  rotateClaw.setPosition(0.57);
        // moveSprocket(3);
        //moveViper(2);



    }

    /*
    // FUNCTIONS DEFINED HERE
     */

    public void moveByJoystick(double seconds, double leftX, double leftY, double rightX ,double wheelTarget) // Moves the robot by simulating a joystick
    {
        // Just check out https://gm0.org/en/latest/docs/software/tutorials/mecanum-drive.html for explanation
        double x = -leftX;
        double y = -leftY;
        double rotation = -rightX;
        double FL = (y + x + rotation);
        double FR = (y - x - rotation);
        double BL = (y - x + rotation);
        double BR = (y + x - rotation);

        int TFL = (int) wheelTarget;
        int TFR = (int) wheelTarget;
        int TBR = (int) wheelTarget;
        int TBL = (int) wheelTarget;


        newFL = (int) Math.round(newFL + FL * wheelTarget);
        newFR = (int) Math.round(newFR + (FR * wheelTarget));
        newBL = (int) Math.round(newBL + BL * wheelTarget);
        newBR = (int) Math.round(newBR + BR * wheelTarget);

//        wheelFL.setTargetPosition(TFL);
////        wheelFR.setTargetPosition(TFR);
////        wheelBL.setTargetPosition(TBL);
////        wheelBR.setTargetPosition(TBR);

        wheelFL.setTargetPosition(newFL);
        wheelFR.setTargetPosition(newFR);
        wheelBL.setTargetPosition(newBL);
        wheelBR.setTargetPosition(newBR);

        telemetry.addData("FL is busy", wheelFL.isBusy());
        telemetry.addData("FR is busy", wheelFR.isBusy());
        telemetry.addData("BL is busy", wheelBL.isBusy());
        telemetry.addData("BR is busy", wheelBR.isBusy());
        telemetry.addData("BL is at target", wheelBL.getTargetPosition());
        telemetry.addData("FL is at target", wheelFL.getTargetPosition());
        telemetry.addData("FR is at target", wheelFR.getTargetPosition());
        telemetry.addData("BR is at target", wheelBR.getTargetPosition());
        telemetry.update();


//        wheelFL.setPower(0.8);
//        wheelFR.setPower(0.8);
//        wheelBL.setPower(0.8);
//        wheelBR.setPower(0.8);


        runtime.reset();
        while ((wheelFL.isBusy() || wheelFR.isBusy() || wheelBL.isBusy() || wheelBR.isBusy()) && runtime.seconds() < seconds) {
            // Debugging telemetry can go here
        }
        if (runtime.seconds() >= seconds) {
            telemetry.addData("Timeout", "Motors did not reach target in time");
            telemetry.update();
        }

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
    public  void reset(){
        wheelFL.setTargetPosition(0); wheelFR.setTargetPosition(0);
        wheelBL.setTargetPosition(0); wheelBR.setTargetPosition(0);
    }










    public void moveForward(double seconds, double power , double wheelTarget)
    {
        moveByJoystick(seconds, 0, power, 0 , wheelTarget);
    }

    public void rotate(double seconds, double power, double wheelTarget)
    {
        moveByJoystick(seconds, 0, 0, power, wheelTarget );
    }

    public void strafe(double seconds, double power , double forwardwheelTarget , double negativewheelTarget)
    {
        //  moveByStrafe(seconds, power, 0, 0 , forwardwheelTarget , negativewheelTarget); ;
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