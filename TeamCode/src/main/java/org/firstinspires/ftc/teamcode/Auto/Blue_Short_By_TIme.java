
package org.firstinspires.ftc.teamcode.Auto;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.acmerobotics.dashboard.config.Config;

@Autonomous(name="Robot: Auto Drive By Time to blue", group="Robot")

@Config
public class Blue_Short_By_TIme extends LinearOpMode {

    private DcMotorEx wheelFL;
    private DcMotorEx wheelFR;
    private DcMotorEx wheelBL;
    private DcMotorEx wheelBR;
    private ElapsedTime  runtime = new ElapsedTime();


    static final double  FORWARD_SPEED = -0.6;
    static final double     TURN_SPEED    = 0.5;
    @Override
    public void runOpMode() throws InterruptedException {
        //Motors
        wheelFL = hardwareMap.get(DcMotorEx.class, "wheelFL");
        wheelFR = hardwareMap.get(DcMotorEx.class, "wheelFR");
        wheelBL = hardwareMap.get(DcMotorEx.class, "wheelBL");
        wheelBR = hardwareMap.get(DcMotorEx.class, "wheelBR");

        wheelFL.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        wheelFR.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        wheelBL.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        wheelBR.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);


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
        wheelFL.setPower(FORWARD_SPEED);
        wheelFR.setPower(FORWARD_SPEED);
        wheelBR.setPower(FORWARD_SPEED);
        wheelBL.setPower(FORWARD_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.2)) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        while (opModeIsActive()&& (runtime.seconds() < 1.2)){

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
