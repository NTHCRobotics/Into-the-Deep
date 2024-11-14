package org.firstinspires.ftc.teamcode.Auto;

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

// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name="Red_long ",group = "Robot")


public class Red_Long extends LinearOpMode{

    private DcMotorEx wheelFL;
    private DcMotorEx wheelFR;
    private DcMotorEx wheelBL;
    private DcMotorEx wheelBR;
    private DcMotorEx viper;

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
        viper = hardwareMap.get(DcMotorEx.class, "viper");


        wheelFL.setDirection(DcMotorSimple.Direction.REVERSE);//REVERSE
        wheelFR.setDirection(DcMotorSimple.Direction.FORWARD);//FORWARD
        wheelBL.setDirection(DcMotorSimple.Direction.FORWARD);//FORWARD
        wheelBR.setDirection(DcMotorSimple.Direction.REVERSE);//REVERSE


        wheelFL.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        wheelFR.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        wheelBL.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        wheelBR.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);


        // Viper Encoder
        viper.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        viper.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        viper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        viper.setTargetPositionTolerance(50);
        viper.setTargetPosition(50);
        viper.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
// Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(11.8, 61.7, Math.toRadians(90)));
        Action trajectoryAction1;
        trajectoryAction1 = drive.actionBuilder(drive.pose)
                // Pre load Sample
                .splineTo(new Vector2d(-33,-35.7), Math.toRadians(120))
                .splineTo(new Vector2d(-55, -54), Math.toRadians(230))
                // first sample
                .splineTo(new Vector2d(-47, -33), Math.toRadians(90))
                .splineTo(new Vector2d(-55, -54), Math.toRadians(230))
                .waitSeconds(3)
                // Second sample
                .splineTo(new Vector2d(-57., -34), Math.toRadians(90))
                .splineTo(new Vector2d(-55, -54), Math.toRadians(230))
                .waitSeconds(3)
                // Third sample
                .splineTo(new Vector2d(-35, -38), Math.toRadians(90))
                .splineTo(new Vector2d(-54, -25), Math.toRadians(180))
                .splineTo(new Vector2d(-55, -54), Math.toRadians(230))
                .build();
        
        
        Actions.runBlocking(
                new SequentialAction(
                        trajectoryAction1
                )
              );


    }
}
