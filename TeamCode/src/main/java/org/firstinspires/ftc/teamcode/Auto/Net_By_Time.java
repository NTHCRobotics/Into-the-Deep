package org.firstinspires.ftc.teamcode.Auto;





import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.acmerobotics.dashboard.config.Config;

@Autonomous(name="spooderman", group="Robot")

@Config
public class Net_By_Time extends LinearOpMode {

    private DcMotorEx wheelFL;
    private DcMotorEx wheelFR;
    private DcMotorEx wheelBL;
    private DcMotorEx wheelBR;
    private Servo Claw;
    private Servo RotationalClaw;
    private  DcMotorEx Rocket;
    private ElapsedTime runtime = new ElapsedTime();


    static final double FORWARD_SPEED = -0.5;
    static final double TURN_SPEED = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {
        //Motors
        wheelFL = hardwareMap.get(DcMotorEx.class, "wheelFL");
        wheelFR = hardwareMap.get(DcMotorEx.class, "wheelFR");
        wheelBL = hardwareMap.get(DcMotorEx.class, "wheelBL");
        wheelBR = hardwareMap.get(DcMotorEx.class, "wheelBR");
        Claw = hardwareMap.get(Servo.class, "claw");
        RotationalClaw = hardwareMap.get(Servo.class, "rotateClaw");
        Rocket = hardwareMap.get(DcMotorEx.class, "rocket");
        wheelFL.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        wheelFR.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        wheelBL.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        wheelBR.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        Rocket.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Rocket.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Rocket.setDirection(DcMotorSimple.Direction.FORWARD);
        Rocket.setTargetPosition(0);
        Rocket.setVelocity(700);


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

        // Step 1:  Drive forward for 3 seconds

        Claw.setPosition(0.55);
        RotationalClaw.setPosition(0.1);
        Rocket.setTargetPosition(55);
        wheelFL.setPower(FORWARD_SPEED);
        wheelFR.setPower(FORWARD_SPEED);
        wheelBR.setPower(FORWARD_SPEED);
        wheelBL.setPower(FORWARD_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.65)) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        while (opModeIsActive() && (runtime.seconds() < 1.2)) {

        }


        // Step 4:  Stop
        wheelBL.setPower(0);
        wheelBR.setPower(0);
        wheelFR.setPower(0);
        wheelFL.setPower(0);
        Claw.setPosition(1);


        telemetry.addData("Path", "Complete");
        telemetry.update();

        sleep(1000);


    }
}
