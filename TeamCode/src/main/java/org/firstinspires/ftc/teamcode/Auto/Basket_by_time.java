
package org.firstinspires.ftc.teamcode.Auto;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.acmerobotics.dashboard.config.Config;

@Autonomous(name="Basket_by_time", group="Robot")

@Config
public class Basket_by_time extends LinearOpMode {

    private DcMotorEx wheelFL;
    private DcMotorEx wheelFR;
    private DcMotorEx wheelBL;
    private DcMotorEx wheelBR;
    private DcMotorEx viper; //Vertical lift mechanism
    private DcMotorEx Rocket; // Motor for rotate the Vertical lift
    private Servo Claw;
    private Servo RotationalClaw; // Second CLaw
    private ElapsedTime  runtime = new ElapsedTime();
    private int[] armLevelPosition = {0, 1200, 2500, 3250,};
    private int armLevel;

    static final double  FORWARD_SPEED = -0.5;
    static final double Backward_Speed = 0.5;

    public  void Right_Turn (){
        wheelFL.setPower(0.5);
        wheelFR.setPower(-0.5);

        wheelBL.setPower(0.5);
        wheelFR.setPower(-0.5);

    }
    public  void Left_turn(){
        wheelFL.setPower(-0.2);
        wheelFR.setPower(0.2);

        wheelBL.setPower(-0.2);
        wheelBR.setPower(0.2);
    }
    public  void Strafe_Left(){
        wheelFL.setPower(FORWARD_SPEED);
        wheelFR.setPower(Backward_Speed);
        wheelBR.setPower(FORWARD_SPEED);
        wheelBL.setPower(Backward_Speed);


    }
    public  void Strafe_Right(){
       wheelFL.setPower(Backward_Speed);
       wheelFR.setPower(FORWARD_SPEED);
       wheelBL.setPower(FORWARD_SPEED);
       wheelBR.setPower(Backward_Speed);
        }

    public void Move_Forward(){
        wheelFR.setPower(FORWARD_SPEED);
        wheelFL.setPower(FORWARD_SPEED);
        wheelBL.setPower(FORWARD_SPEED);
        wheelBR.setPower(FORWARD_SPEED);
    }
    public  void Move_Backward(){
        wheelFR.setPower(Backward_Speed);
        wheelFL.setPower(Backward_Speed);
        wheelBL.setPower(Backward_Speed);
        wheelBR.setPower(Backward_Speed);
    }
    public void Wheel_Stop(){
        wheelBL.setPower(0);
        wheelBR.setPower(0);
        wheelFR.setPower(0);
        wheelFL.setPower(0);
    }







    @Override
    public void runOpMode() throws InterruptedException {
        //Motors
        wheelFL = hardwareMap.get(DcMotorEx.class, "wheelFL");
        wheelFR = hardwareMap.get(DcMotorEx.class, "wheelFR");
        wheelBL = hardwareMap.get(DcMotorEx.class, "wheelBL");
        wheelBR = hardwareMap.get(DcMotorEx.class, "wheelBR");
        Claw = hardwareMap.get(Servo.class, "claw");
        viper = hardwareMap.get(DcMotorEx.class, "viper");
        Rocket = hardwareMap.get(DcMotorEx.class, "rocket");
        RotationalClaw = hardwareMap.get(Servo.class, "rotateClaw");


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
        viper.setVelocity(10000);
        viper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Sprocket Encoder
        Rocket.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Rocket.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Rocket.setDirection(DcMotorSimple.Direction.FORWARD);
        Rocket.setTargetPosition(0);
        Rocket.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Rocket.setVelocity(2000);



        wheelFL.setDirection(DcMotorSimple.Direction.FORWARD);//REVERSE
        wheelFR.setDirection(DcMotorSimple.Direction.REVERSE);//FORWARD
        wheelBL.setDirection(DcMotorSimple.Direction.FORWARD);//FORWARD
        wheelBR.setDirection(DcMotorSimple.Direction.REVERSE);//REVERSE

        wheelFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wheelFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wheelBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wheelBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        telemetry.addData("Rocket Position", Rocket.getCurrentPosition());
        telemetry.update();




        waitForStart();
// Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path, ensuring that the Auto mode has not been stopped along the way

        // Step 1:  Drive forward for 1 seconds
        Claw.setPosition(0.7);
       Move_Backward();
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.3)) {
            telemetry.addData("Path 1", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        // Will Strafe in some direction
        runtime.reset();
        Strafe_Right();
        while (opModeIsActive()&& (runtime.seconds() < 0.5)){
                telemetry.addData("Path 1 Strafe ", "Leg 2",runtime.seconds());
                telemetry.update();
        }
        runtime.reset();
        // Turn for 0.2 seconds

        wheelFL.setPower(-FORWARD_SPEED);
        wheelFR.setPower(-Backward_Speed);
        wheelBR.setPower(-FORWARD_SPEED);
        wheelBL.setPower(-Backward_Speed);
        while (opModeIsActive()&& (runtime.seconds() < 1.6)){
            telemetry.addData("Path 1 Turn ", "Leg 3",runtime.seconds());
        }


        wheelBL.setPower(0);
        wheelBR.setPower(0);
        wheelFR.setPower(0);
        wheelFL.setPower(0);

        runtime.reset();
        Rocket.setTargetPosition(970);
        while (opModeIsActive()&& (runtime.seconds() < 2.1)){
            telemetry.addData("Path 1 Sprocket Scoring Postion ", "Leg 4",runtime.seconds());
        }
        runtime.reset();
        RotationalClaw.setPosition(.88);
        while (opModeIsActive()&& (runtime.seconds() < 2)){
            telemetry.addData("Path 1 Viper Scoring Postion ", "Leg 5",runtime.seconds());

        }

        viper.setTargetPosition(3250);
        while (opModeIsActive()&& (runtime.seconds() < 5)){
            telemetry.addData("Path 1 Viper Scoring Postion ", "Leg 5",runtime.seconds());

        }
        runtime.reset();
        Move_Backward();

        while (opModeIsActive()&& (runtime.seconds() < 0.15)){
            telemetry.addData("Path 1 Sprocket Scoring Postion ", "Leg 4",runtime.seconds());
        }
        runtime.reset();
        wheelBL.setPower(0);
        wheelBR.setPower(0);
        wheelFR.setPower(0);
        wheelFL.setPower(0);
        while (opModeIsActive()&& (runtime.seconds() < 0.2)){
            telemetry.addData("Path 1 Sprocket Scoring Postion ", "Leg 4",runtime.seconds());
        }




        runtime.reset();
        RotationalClaw.setPosition(0.5);

        while (opModeIsActive()&& (runtime.seconds() < 1.5)) {
            telemetry.addData("Claw Command ", "Leg 6", runtime.seconds());
        }
        Claw.setPosition(1);
        runtime.reset();
        while (opModeIsActive()&& (runtime.seconds() < 1.5)) {
            telemetry.addData("Claw Command ", "Leg 6", runtime.seconds());
        }
        runtime.reset();
        RotationalClaw.setPosition(0.68);
        viper.setTargetPosition(0);
        while (opModeIsActive()&& (runtime.seconds() < 1.2)) {
            telemetry.addData("Path 1 Viper Rest", "Leg 7", runtime.seconds());
        }
        runtime.reset();
        

        Rocket.setTargetPosition(0);
        Claw.setPosition(1);
        while (opModeIsActive()&& (runtime.seconds() < 0.5)) {
            telemetry.addData("Path 1 Sprocket Rest Postion ", "Leg 8", runtime.seconds());
        }







        // Step 4:  Stop
        wheelBL.setPower(0);
        wheelBR.setPower(0);
        wheelFR.setPower(0);
        wheelFL.setPower(0);


        telemetry.addData("Path", "Complete");
        telemetry.update();

        sleep(1000);



    }

}
