package org.firstinspires.ftc.teamcode.Auto;



import com.acmerobotics.roadrunner.Trajectory;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import  com.acmerobotics.roadrunner.TrajectoryBuilder;

// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.teamcode.MecanumDrive;



@Autonomous(name="RedShort", group="Robot")


public class    Red_Short extends LinearOpMode {

    private DcMotorEx wheelFL;
    private DcMotorEx wheelFR;
    private DcMotorEx wheelBL;
    private DcMotorEx wheelBR;
    private  Servo Claw;
    private ElapsedTime runtime = new ElapsedTime();


    static final double FORWARD_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {
        //Motors
        wheelFL = hardwareMap.get(DcMotorEx.class, "wheelFL");
        wheelFR = hardwareMap.get(DcMotorEx.class, "wheelFR");
        wheelBL = hardwareMap.get(DcMotorEx.class, "wheelBL");
        wheelBR = hardwareMap.get(DcMotorEx.class, "wheelBR");

        //Servos
        Claw = hardwareMap.get(Servo.class, "Claw");

        wheelFL.setDirection(DcMotorSimple.Direction.FORWARD);//FORWARD
        wheelFR.setDirection(DcMotorSimple.Direction.REVERSE);//REVERSE
        wheelBL.setDirection(DcMotorSimple.Direction.FORWARD);//FORWARD
        wheelBR.setDirection(DcMotorSimple.Direction.REVERSE);//REVERSE


        wheelFL.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        wheelFR.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        wheelBL.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        wheelBR.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        waitForStart();
// Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(-12.8, -62.7, Math.toRadians(90)));


    }
}






