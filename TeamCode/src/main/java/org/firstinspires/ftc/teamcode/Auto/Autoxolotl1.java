
package org.firstinspires.ftc.teamcode.Auto;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.acmerobotics.dashboard.config.Config;

@Autonomous(name="JeffTheLandShark", group="Omarxolotl")

@Config
public class Autoxolotl1 extends LinearOpMode {
    //Setting Variables
    // Extras
    private int[] wheelTicks = {0, 0, 0, 0}; // FL = 0, FR = 1, BL = 2, BR = 3
    private final ElapsedTime runtime = new ElapsedTime();  // Timer, I just copy pasted, don't ask questions
    private double speedMod;
    private int[] viperSlideTargets = {0, 1600, 100,2060, 2800,2120, 1900};

    private  int[] sprocketTargets  = {0, 70 , 350 , 750, 310,100};


    private double CLawGrip = 0.45;

    private int viperlevel;


    //Motors
    private DcMotorEx wheelFL; // Front left wheel
    private DcMotorEx wheelFR; // Front right wheel
    private DcMotorEx wheelBL; // Back left wheel
    private DcMotorEx wheelBR; // Back right wheel
    private DcMotorEx SwyftSlideJr;
    private DcMotorEx SwyftSlide;
    private DcMotorEx rocket; // Motor for rotate the Vertical lift

    // Wheel Ticks
    private int newFL = 0;
    private int newFR = 0;
    private int newBL = 0;
    private int newBR = 0;

    //Servos
    private Servo claw; // Opening and closing of the claw
    private Servo rotateClaw; // Rotates the claw up and down

    private Servo rollClaw;


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


        SwyftSlide = hardwareMap.get(DcMotorEx.class, "SwyftSlide");
        SwyftSlideJr = hardwareMap.get(DcMotorEx.class, "SwyftSlideJr");
        rocket = hardwareMap.get(DcMotorEx.class, "rocket");


        //------------SERVOS////
        claw = hardwareMap.get(Servo.class, "claw");
        rotateClaw = hardwareMap.get(Servo.class, "rotateClaw");
        rollClaw = hardwareMap.get(Servo.class, "rollClaw");

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


        SwyftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        SwyftSlide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        SwyftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        SwyftSlide.setTargetPositionTolerance(50);
        SwyftSlide.setTargetPosition(0);
        SwyftSlide.setDirection(DcMotorSimple.Direction.FORWARD);
        SwyftSlide.setVelocity(2000);

        // SwyftSlideJr Encoder

        SwyftSlideJr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        SwyftSlideJr.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        SwyftSlideJr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        SwyftSlideJr.setTargetPositionTolerance(50);
        SwyftSlideJr.setTargetPosition(0);
        SwyftSlideJr.setDirection(DcMotorSimple.Direction.REVERSE);
        SwyftSlideJr.setVelocity(2000);


        // Sprocket Stuff
        rocket.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rocket.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rocket.setDirection(DcMotorSimple.Direction.REVERSE);
        rocket.setTargetPosition(0);
        rocket.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rocket.setVelocity(1200);


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
    moveByJoystick(0.,0,1,0,700);
        setRotateClaw(0.35,0.2);
        moveSprocket(2);
        moveSwyftSlides(5);
        //moveSprocket(4);
        moveSwyftSlides(6);
        setRotateClaw(0.25,0.2);
    //moveByJoystick(0.3,-1,0,0,900);
/*
        setClaw(0.45, 0.4);
        setRotateClaw(0.1, 0.6);
        moveByJoystick(0.6, 0, -1, 0, 520
        );
        moveByJoystick(0.5, -1, 0, 0, 540);
        moveByJoystick(0.5, 0, 0, -1, 450);
        moveByJoystick(0.3, 0, -1, 0, 225);
        moveSprocket(3);
        moveSwyftSlides(4);
        setRotateClaw(0.45, 1);
        setClaw(1, 0.6);
        setRotateClaw(0, 0.4);
        moveSwyftSlides(0);
        moveSprocket(1);

        //First Sample

        moveByJoystick(0.5, 0, 0, -1, 527);
        moveSwyftSlides(2);
        setClaw(0.45, 0.4);
        setRotateClaw(0.1, 0.2);
        moveSwyftSlides(0);

        moveByJoystick(0.5, 0, 0, 1, 527);
        moveSprocket(3);
        moveSwyftSlides(4);
        setRotateClaw(0.45, 0.4);
        setClaw(1, 0.2);
        setRotateClaw(0., 0.4);
        moveSwyftSlides(0);
        moveSprocket(1);

        //Second Sample

        moveByJoystick(0.6, 0, 0, -1, 760);

        moveSwyftSlides(3);
        setClaw(0.45, 0.3);
        moveSwyftSlides(0);
        moveByJoystick(0.6, 0, 0, 1, 570);
        moveSprocket(3);
        moveSwyftSlides(4);
        setRotateClaw(0.45, 0.4);
        setClaw(1, 0.2);
        setRotateClaw(0, 0.4);
        moveSwyftSlides(0);
        moveSprocket(1);
*/

        //Thrid Sample


    }

    /*
    // FUNCTIONS DEFINED HERE
     */

    public void moveByJoystick(double seconds, double leftX, double leftY, double rightX, double wheelTarget) // Moves the robot by simulating a joystick
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


    public void positionSwyftSlides(int SwyftSlidesTarget) {

        SwyftSlide.setTargetPosition(viperSlideTargets[SwyftSlidesTarget]);
        SwyftSlideJr.setTargetPosition(viperSlideTargets[SwyftSlidesTarget]);
        viperWait();

    }

    public void setClaw(double clawTarget, double seconds) {
        claw.setPosition(clawTarget);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < seconds)) {
            // Nothing?
        }
        runtime.reset();

    }

    public void setRotateClaw(double rotateClawTarget , double seconds){
        rotateClaw.setPosition(rotateClawTarget);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < seconds))
        {
            // Nothing?
        }
        runtime.reset();

    }
    public void setRollClaw(double rollClawTarget , double seconds){
        rollClaw.setPosition(rollClawTarget);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < seconds))
        {
            // Nothing?
        }
        runtime.reset();

    }

    public void Pickup(double seconds) {
        rotateClaw.setPosition(0);
        rollClaw.setPosition(0);
        claw.setPosition(0);

    }

    public void Score(double seconds) {
        rollClaw.setPosition(1);
        rotateClaw.setPosition(0.45);
        claw.setPosition(1);
    }

    public void PreScore(double seconds) {
        claw.setPosition(1);
        rollClaw.setPosition(0.5);
    }


    public void positionSprocket(int sproketTarget) {
        rocket.setTargetPosition(sprocketTargets[sproketTarget]);
        sprocketWait();
    }


    public void wait(double seconds) // Waits the amount of seconds specified
    {
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < seconds)) {
            // Nothing?
        }
        runtime.reset();


    }

    public void viperWait() // Waits the amount of seconds specified
    {
        runtime.reset();
        while (SwyftSlide.isBusy() && SwyftSlideJr.isBusy()) {
            // Nothing?
        }
        runtime.reset();


    }

    public void sprocketWait() // Waits the amount of seconds specified
    {
        runtime.reset();
        while (rocket.isBusy()) {
            // Nothing?
        }
        runtime.reset();


    }

    public void reset() {
        wheelFL.setTargetPosition(0);
        wheelFR.setTargetPosition(0);
        wheelBL.setTargetPosition(0);
        wheelBR.setTargetPosition(0);
    }


    public void moveForward(double seconds, double power, double wheelTarget) {
        moveByJoystick(seconds, 0, power, 0, wheelTarget);
    }

    public void rotate(double seconds, double power, double wheelTarget) {
        moveByJoystick(seconds, 0, 0, power, wheelTarget);
    }

    public void strafe(double seconds, double power, double forwardwheelTarget, double negativewheelTarget) {
        //  moveByStrafe(seconds, power, 0, 0 , forwardwheelTarget , negativewheelTarget); ;
    }

    public void moveSwyftSlides(int SwyftSlideTarget) {
        positionSwyftSlides(SwyftSlideTarget);
    }


    public void moveSprocket(int sproketTarget) {
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
