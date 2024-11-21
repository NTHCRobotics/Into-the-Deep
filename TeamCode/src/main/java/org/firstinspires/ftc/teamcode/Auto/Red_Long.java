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
import com.acmerobotics.roadrunner.TrajectoryBuilder;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import

@Autonomous(name="Red_long ",group = "Robot")


public class Red_Long extends LinearOpMode {

    private DcMotorEx wheelFL;
    private DcMotorEx wheelFR;
    private DcMotorEx wheelBL;
    private DcMotorEx wheelBR;


    private ElapsedTime runtime = new ElapsedTime();
    private final Pose2d initialPose = new Pose2d(-24, -62.5, Math.toRadians(180));
    private Action path0, path1, path2, path3;

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

        TrajectoryActionBuilder tab0 = drive.actionBuilder(initialPose)
                .splineTo(new Vector2d(-50, -58), Math.toRadians(225));
        TrajectoryActionBuilder tab1 = drive.actionBuilder(new Pose2d(-50, -58, Math.toRadians(225)))
                .setReversed(true)
                .splineTo(new Vector2d(-34, -40), Math.toRadians(0))
                .setReversed(false)
                .splineTo(new Vector2d( -46, -60), Math.toRadians(90))
                .lineToY(62);
        TrajectoryActionBuilder tab2 = drive.actionBuilder(new Pose2d(-46, -62, Math.toRadians(90)))
                .setReversed(true)
                .splineTo(new Vector2d(-34, -48), Math.toRadians(0))
                .setReversed(false)
                .splineTo(new Vector2d(-58, -49), Math.toRadians(225));
        TrajectoryActionBuilder tab3 = drive.actionBuilder(new Pose2d(-58, -49, Math.toRadians(225)))
                .setReversed(true)
                .splineTo(new Vector2d(-34, -48), Math.toRadians(0))
                .setReversed(false)
                .splineTo(new Vector2d(-30, -34), Math.toRadians(0));
        path0 = tab0.build();
        path1 = tab1.build();
        path2 = tab2.build();
        path3 = tab3.build();
    }

    }

