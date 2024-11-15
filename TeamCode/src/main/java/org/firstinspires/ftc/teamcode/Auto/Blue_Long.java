package org.firstinspires.ftc.teamcode.Auto;




import androidx.annotation.NonNull;

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



@Autonomous(name="BlueLong", group="Robot")


public class    Blue_Long extends LinearOpMode {

    private DcMotorEx wheelFL;
    private DcMotorEx wheelFR;
    private DcMotorEx wheelBL;
    private DcMotorEx wheelBR;
    private DcMotorEx viper;
    private Servo claw;
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
            return new
                    OpenClaw();
        }
    }
    @Override
    public void runOpMode() throws InterruptedException {
        //Motors
        wheelFL = hardwareMap.get(DcMotorEx.class, "wheelFL");
        wheelFR = hardwareMap.get(DcMotorEx.class, "wheelFR");
        wheelBL = hardwareMap.get(DcMotorEx.class, "wheelBL");
        wheelBR = hardwareMap.get(DcMotorEx.class, "wheelBR");
        Claw claw = new Claw(hardwareMap);
        Viper viper = new Viper(hardwareMap);

        wheelFL.setDirection(DcMotorSimple.Direction.REVERSE);//REVERSE
        wheelFR.setDirection(DcMotorSimple.Direction.FORWARD);//FORWARD
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


        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(11.8, 61.7, Math.toRadians(90)));
        Action trajectoryAction1;
        trajectoryAction1 = drive.actionBuilder(drive.pose)
                // Pre load Sample
                .splineTo(new Vector2d(52 ,54), Math.toRadians(40))
                .build();
        Action trajectoryAction2;
        trajectoryAction2 = drive.actionBuilder(drive.pose)
                // first sample
                .splineTo(new Vector2d(43, 45), Math.toRadians(60))
                .splineTo(new Vector2d(46, 39), Math.toRadians(100))
                .splineTo(new Vector2d(52 ,54), Math.toRadians(40))
                .build();
        Action trajeectoryAction3;
        trajeectoryAction3 = drive.actionBuilder(drive.pose)

                // second sample
                .splineTo(new Vector2d(57.5, 36), Math.toRadians(90))
                .splineTo(new Vector2d(52 ,54), Math.toRadians(40))
                .build();
        Action trajectoryAction4;
        trajectoryAction4 = drive.actionBuilder(drive.pose)
                // third sample
                .splineTo(new Vector2d(50, 24), Math.toRadians(30))
                .splineTo(new Vector2d(52 ,54), Math.toRadians(40))
                .build();


        Actions.runBlocking(
                new SequentialAction(
                        trajectoryAction1,
                        viper.ViperDown(),
                        claw.openClaw(),
                        viper.ViperDown()


                )
        );

    }
}
