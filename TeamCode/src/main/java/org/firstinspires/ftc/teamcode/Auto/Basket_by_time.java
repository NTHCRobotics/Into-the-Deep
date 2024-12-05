
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

    static final double  FORWARD_SPEED = -0.5;
    static final double Backward_Speed = 0.5;
    static final double     Right_TURN_SPEED    = 0.5;
    static  final double Left_Turn_Speed = -0.5;
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
        RotationalClaw = hardwareMap.get(Servo.class, "rotationalClaw");


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
        Rocket.setDirection(DcMotorSimple.Direction.FORWARD);
        Rocket.setTargetPosition(0);
        Rocket.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        wheelFL.setDirection(DcMotorSimple.Direction.FORWARD);//REVERSE
        wheelFR.setDirection(DcMotorSimple.Direction.REVERSE);//FORWARD
        wheelBL.setDirection(DcMotorSimple.Direction.FORWARD);//FORWARD
        wheelBR.setDirection(DcMotorSimple.Direction.REVERSE);//REVERSE


        waitForStart();
// Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path, ensuring that the Auto mode has not been stopped along the way

        // Step 1:  Drive forward for 1 seconds
        Claw.setPosition(0.65);
        wheelFL.setPower(FORWARD_SPEED);
        wheelFR.setPower(FORWARD_SPEED);
        wheelBR.setPower(FORWARD_SPEED);
        wheelBL.setPower(FORWARD_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.0)) {
            telemetry.addData("Path 1", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        // Will Strafe in some direction
        wheelFL.setPower(FORWARD_SPEED);
        wheelFR.setPower(Backward_Speed);
        wheelBR.setPower(FORWARD_SPEED);
        wheelBL.setPower(Backward_Speed);
        while (opModeIsActive()&& (runtime.seconds() < 0.2)){
                telemetry.addData("Path 1 Strafe ", "Leg 2",runtime.seconds());
        }
        // Turn for 0.2 seconds
        wheelFL.setPower(Left_Turn_Speed);

        wheelBR.setPower(Left_Turn_Speed);
        while (opModeIsActive()&& (runtime.seconds() < 0.2)){
            telemetry.addData("Path 1 Turn ", "Leg 3",runtime.seconds());
        }
        Rocket.setTargetPosition(970);
        while (opModeIsActive()&& (runtime.seconds() < 0.2)){
            telemetry.addData("Path 1 Sprocket Scoring Postion ", "Leg 4",runtime.seconds());
        }
        RotationalClaw.setPosition(.43);
        viper.setTargetPosition(1500);
        while (opModeIsActive()&& (runtime.seconds() < 1.2)){
            telemetry.addData("Path 1 Viper Scoring Postion ", "Leg 5",runtime.seconds());

        }
        RotationalClaw.setPosition(0);
        Claw.setPosition(0.65);
        while (opModeIsActive()&& (runtime.seconds() < 0.5)) {
            telemetry.addData("Claw Command ", "Leg 6", runtime.seconds());
        }

        RotationalClaw.setPosition(0.43);
        viper.setTargetPosition(0);
        while (opModeIsActive()&& (runtime.seconds() < 1.2)) {
            telemetry.addData("Path 1 Viper Rest", "Leg 7", runtime.seconds());
        }

        Rocket.setTargetPosition(0);
        Claw.setPosition(1);
        while (opModeIsActive()&& (runtime.seconds() < 0.5)) {
            telemetry.addData("Path 1 Sprocket Rest Postion ", "Leg 8", runtime.seconds());
        }

        // Seq 2
        wheelFL.setPower(Right_TURN_SPEED);

        wheelBR.setPower(Right_TURN_SPEED);
        while (opModeIsActive()&& (runtime.seconds() < 0.5)) {
            telemetry.addData("Path 2 Wheel Rotation ", "Path 2 Leg 1", runtime.seconds());
        }

        Rocket.setTargetPosition(210);
        while (opModeIsActive()&& (runtime.seconds() < 0.5)) {
            telemetry.addData("Path 2 Rocket Pick up Postion ", "Path 2 Leg 2", runtime.seconds());
        }

        viper.setTargetPosition(1000);
        while (opModeIsActive()&& (runtime.seconds() < 0.75)) {
            telemetry.addData("Path 2 Rocket Viper PickUp ", "Path 2 Leg 3", runtime.seconds());
        }

        RotationalClaw.setPosition(1);
        Claw.setPosition(0.65);
        while (opModeIsActive()&& (runtime.seconds() < 0.5)) {
            telemetry.addData("Path 2 Rocket Claw Command Pick Up ", "Path 2  Leg 4", runtime.seconds());
        }
        viper.setTargetPosition(0);
        while (opModeIsActive()&& (runtime.seconds() < 0.5)) {
            telemetry.addData("Path 2 Viper Rest ", "Path 2 Leg 5", runtime.seconds());
        }
        wheelFL.setPower(Left_Turn_Speed);

        wheelBR.setPower(Left_Turn_Speed);

        while (opModeIsActive()&& (runtime.seconds() < 0.5)) {
            telemetry.addData("Path 2 Viper Rest ", "Path 2 Leg 5", runtime.seconds());
        }
      Rocket.setTargetPosition(970);
        RotationalClaw.setPosition(0.43);
        while (opModeIsActive()&& (runtime.seconds() < 0.5)) {
            telemetry.addData("Path 2 Rocket Score  Postion ", "Path 2 Leg 6", runtime.seconds());
        }

      viper.setTargetPosition(2500);
        while (opModeIsActive()&& (runtime.seconds() < 0.5)) {
            telemetry.addData("Path 2 Viper Score Postion ", "Path 2 Leg 7", runtime.seconds());
        }
      RotationalClaw.setPosition(0);
      Claw.setPosition(0.65);
        while (opModeIsActive()&& (runtime.seconds() < 0.5)) {
            telemetry.addData("Path 2 Claw Score Command ", "Path 2 Leg 8", runtime.seconds());
        }

      RotationalClaw.setPosition(0.43);
      viper.setTargetPosition(0);

        while (opModeIsActive()&& (runtime.seconds() < 0.5)) {
            telemetry.addData("Path 2 Viper Rest ", "Path 2 Leg 9", runtime.seconds());
        }

       // Seq 3
        Rocket.setTargetPosition(210);

        while (opModeIsActive()&& (runtime.seconds() < 0.5)) {
            telemetry.addData("Path 2 Rocket Pick Up ", "Path 3 Leg 1", runtime.seconds());
        }

        wheelFL.setPower(Right_TURN_SPEED);

        wheelBR.setPower(Right_TURN_SPEED);

        while (opModeIsActive()&& (runtime.seconds() < 0.5)) {
            telemetry.addData("Path 2 Wheel Rotation ", "Path 3 Leg 2", runtime.seconds());
        }

        Rocket.setTargetPosition(210);
        while (opModeIsActive()&& (runtime.seconds() < 0.5)) {
            telemetry.addData("Path 2 Rocket Pick up Postion ", "Path 3 Leg 3", runtime.seconds());
        }

        viper.setTargetPosition(1000);
        while (opModeIsActive()&& (runtime.seconds() < 0.75)) {
            telemetry.addData("Path 2 Rocket Viper PickUp ", "Path 3 Leg 4", runtime.seconds());
        }

        RotationalClaw.setPosition(1);
        Claw.setPosition(0.65);
        while (opModeIsActive()&& (runtime.seconds() < 0.5)) {
            telemetry.addData("Path 2 Rocket Claw Command Pick Up ", "Path 3  Leg 5", runtime.seconds());
        }
        viper.setTargetPosition(0);
        while (opModeIsActive()&& (runtime.seconds() < 0.5)) {
            telemetry.addData("Path 2 Viper Rest ", "Path 2 Leg 5", runtime.seconds());
        }
        wheelFL.setPower(Left_Turn_Speed);

        wheelBR.setPower(Left_Turn_Speed);

        while (opModeIsActive()&& (runtime.seconds() < 0.5)) {
            telemetry.addData("Path 2 Viper Rest ", "Path 3 Leg 6", runtime.seconds());
        }
        Rocket.setTargetPosition(970);
        RotationalClaw.setPosition(0.43);
        while (opModeIsActive()&& (runtime.seconds() < 0.5)) {
            telemetry.addData("Path 2 Rocket Score  Postion ", "Path 3 Leg 7", runtime.seconds());
        }

        viper.setTargetPosition(2500);
        while (opModeIsActive()&& (runtime.seconds() < 0.5)) {
            telemetry.addData("Path 2 Viper Score Postion ", "Path 3 Leg 8", runtime.seconds());
        }
        RotationalClaw.setPosition(0);
        Claw.setPosition(0.65);
        while (opModeIsActive()&& (runtime.seconds() < 0.5)) {
            telemetry.addData("Path 2 Claw Score Command ", "Path 3 Leg 9", runtime.seconds());
        }

        RotationalClaw.setPosition(0.43);
        viper.setTargetPosition(0);

        while (opModeIsActive()&& (runtime.seconds() < 0.5)) {
            telemetry.addData("Path 2 Viper Rest ", "Path 3 Leg 10", runtime.seconds());
        }

        // Seq 4
        // Seq 3
        Rocket.setTargetPosition(210);

        while (opModeIsActive()&& (runtime.seconds() < 0.5)) {
            telemetry.addData("Path 2 Rocket Pick Up ", "Path 4 Leg 1", runtime.seconds());
        }

        wheelFL.setPower(Right_TURN_SPEED);

        wheelBR.setPower(Right_TURN_SPEED);

        while (opModeIsActive()&& (runtime.seconds() < 0.5)) {
            telemetry.addData("Path 2 Wheel Rotation ", "Path 4 Leg 2", runtime.seconds());
        }

        Rocket.setTargetPosition(210);
        while (opModeIsActive()&& (runtime.seconds() < 0.5)) {
            telemetry.addData("Path 2 Rocket Pick up Postion ", "Path 4 Leg 3", runtime.seconds());
        }

        viper.setTargetPosition(1000);
        while (opModeIsActive()&& (runtime.seconds() < 0.75)) {
            telemetry.addData("Path 2 Rocket Viper PickUp ", "Path 4 Leg 4", runtime.seconds());
        }

        RotationalClaw.setPosition(1);
        Claw.setPosition(0.65);
        while (opModeIsActive()&& (runtime.seconds() < 0.5)) {
            telemetry.addData("Path 2 Rocket Claw Command Pick Up ", "Path 4  Leg 5", runtime.seconds());
        }
        viper.setTargetPosition(0);
        while (opModeIsActive()&& (runtime.seconds() < 0.5)) {
            telemetry.addData("Path 2 Viper Rest ", "Path 4 Leg 5", runtime.seconds());
        }
        wheelFL.setPower(Left_Turn_Speed);

        wheelBR.setPower(Left_Turn_Speed);

        while (opModeIsActive()&& (runtime.seconds() < 0.5)) {
            telemetry.addData("Path 2 Viper Rest ", "Path 4 Leg 6", runtime.seconds());
        }
        Rocket.setTargetPosition(970);
        RotationalClaw.setPosition(0.43);
        while (opModeIsActive()&& (runtime.seconds() < 0.5)) {
            telemetry.addData("Path 2 Rocket Score  Postion ", "Path 4 Leg 7", runtime.seconds());
        }

        viper.setTargetPosition(2500);
        while (opModeIsActive()&& (runtime.seconds() < 0.5)) {
            telemetry.addData("Path 2 Viper Score Postion ", "Path 4 Leg 8", runtime.seconds());
        }
        RotationalClaw.setPosition(0);
        Claw.setPosition(0.65);
        while (opModeIsActive()&& (runtime.seconds() < 0.5)) {
            telemetry.addData("Path 2 Claw Score Command ", "Path 4 Leg 9", runtime.seconds());
        }

        RotationalClaw.setPosition(0.43);
        viper.setTargetPosition(0);

        while (opModeIsActive()&& (runtime.seconds() < 0.5)) {
            telemetry.addData("Path 2 Viper Rest ", "Path 4 Leg 10", runtime.seconds());
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
