package org.firstinspires.ftc.teamcode.Auto;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TrajectoryBuilder;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

// RR-specific imports
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


public class Red_Long extends LinearOpMode {

    private DcMotorEx wheelFL;
    private DcMotorEx wheelFR;
    private DcMotorEx wheelBL;
    private DcMotorEx wheelBR;


    private ElapsedTime runtime = new ElapsedTime();


    static final double FORWARD_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;

    public class Viper {
        private DcMotorEx viper;

        public Viper(HardwareMap hardwareMap) {
            viper = hardwareMap.get(DcMotorEx.class, "viper");
            viper.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            viper.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            viper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            viper.setTargetPositionTolerance(50);
            viper.setTargetPosition(50);
            viper.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        public class ViperUp implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                viper.setTargetPosition(2265);
                return false;
            }
        }

        public Action ViperUp() {
            return new ViperUp();
        }

        public class ViperDown implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                viper.setTargetPosition(0);
                return false;
            }
        }

        public Action ViperDown() {
            return new ViperDown();
        }
    }
    public class Rocket {
        private DcMotorEx Rocket;

        public Rocket(HardwareMap hardwareMap) {
            Rocket = hardwareMap.get(DcMotorEx.class, "rocket");
            Rocket.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            Rocket.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Rocket.setDirection(DcMotorSimple.Direction.FORWARD);
            Rocket.setTargetPosition(0);
        }

        public class RocketUp implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                Rocket.setTargetPosition(975);
                return false;
            }
        }

        public Action RocketUp() {
            return new RocketUp();
        }

        public class RocketDown implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                Rocket.setTargetPosition(0);
                return false;
            }
        }

        public Action RocketDown() {
            return new RocketDown();
        }
    }




public class Claw {
        private Servo claw;

        public Claw(HardwareMap hardwareMap) {
            claw = hardwareMap.get(Servo.class, "claw");
        }

        public class CloseClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.setPosition(0.6);
                return false;
            }
        }
        public Action closeClaw() {
            return new CloseClaw();
        }

        public class OpenClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.setPosition(1.0);
                return false;
            }
        }
        public Action openClaw() {
            return new OpenClaw();
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        //Motors
        wheelFL = hardwareMap.get(DcMotorEx.class, "wheelFL");
        wheelFR = hardwareMap.get(DcMotorEx.class, "wheelFR");
        wheelBL = hardwareMap.get(DcMotorEx.class, "wheelBL");
        wheelBR = hardwareMap.get(DcMotorEx.class, "wheelBR");



        wheelFL.setDirection(DcMotorSimple.Direction.REVERSE);//REVERSE
        wheelFR.setDirection(DcMotorSimple.Direction.FORWARD);//FORWARD
        wheelBL.setDirection(DcMotorSimple.Direction.FORWARD);//FORWARD
        wheelBR.setDirection(DcMotorSimple.Direction.REVERSE);//REVERSE


        wheelFL.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        wheelFR.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        wheelBL.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        wheelBR.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        Claw claw = new Claw(hardwareMap);
        Viper viper = new Viper(hardwareMap);



        waitForStart();
// Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(-12.8, -62.7, Math.toRadians(90)));
        MecanumDrive drive1 = new MecanumDrive(hardwareMap, new Pose2d(-55, -54, Math.toRadians(230)));
        MecanumDrive scoresample1 = new MecanumDrive(hardwareMap, new Pose2d(-47, -33, Math.toRadians(90)));
        MecanumDrive scoresample2 = new MecanumDrive(hardwareMap, new Pose2d(-57, -34, Math.toRadians(90)));
        MecanumDrive scoresample3 = new MecanumDrive(hardwareMap, new Pose2d(-54, -25, Math.toRadians(180)));
        Action trajectoryAction1;
        trajectoryAction1 = drive.actionBuilder(drive.pose)
                // Pre load Sample
                .splineTo(new Vector2d(-33,-35.7), Math.toRadians(120))
                .splineTo(new Vector2d(-55, -54), Math.toRadians(230))
                .turnTo(240)
                .build();
                // first sample
       Action trajectoryFirstSample;
        trajectoryFirstSample = drive1.actionBuilder(drive1.pose)
                .splineTo(new Vector2d(-47, -33), Math.toRadians(90))
                .waitSeconds(3)
                .build();
                // Second sample
        Action trajectorySecondSample;
        trajectorySecondSample = drive1.actionBuilder(drive1.pose)
                .splineTo(new Vector2d(-57., -34), Math.toRadians(90))
                .waitSeconds(3)
                .build();
                // Third sample
        Action trajectoryThirdSample;
        trajectoryThirdSample = drive1.actionBuilder(drive1.pose)
                .splineTo(new Vector2d(-35, -38), Math.toRadians(90))
                .splineTo(new Vector2d(-54, -25), Math.toRadians(180))


                .build();


        Action trajectoryScore;
        trajectoryScore = drive1.actionBuilder(scoresample1.pose)
                .splineTo(new Vector2d(-55, -54), Math.toRadians(230))
                .turnTo(240)
                .build();
        Action trajectoryScore2;
        trajectoryScore2 = drive1.actionBuilder(scoresample2.pose)
                .splineTo(new Vector2d(-55, -54), Math.toRadians(230))
                .turnTo(240)
                .build();
        Action trajectoryScore3;
        trajectoryScore3 = drive1.actionBuilder(scoresample3.pose)
                .splineTo(new Vector2d(-55, -54), Math.toRadians(230))
                .turnTo(240)
                .build();

        
        
        Actions.runBlocking(
                new SequentialAction(
                        trajectoryAction1,
                        claw.openClaw()

                )
              );


    }
}
