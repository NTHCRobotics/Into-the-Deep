package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

//Inches used for all units unless otherwise specified
@Autonomous(name="AutoxolotlByTicks", group="Omarxolotl")

//@Config
public class AutoxolotlByTicks extends LinearOpMode {

    private final double WHEEL_RADIUS = Math.PI * 3.78; //Wheel diameter is 3.78 inches, gobilda.com
    private final double TRACK_WIDTH = (14.173 + 17.765) / 2; //Track radius aligned to the center of wheel
    private final double TICKS = 537.7; // YellowJacket 5203 ticks/rotation from gobila website

    private final double SPEED = 0.6;


    //Wheels
    private DcMotorEx wheelFL; // Front left wheel
    private DcMotorEx wheelFR; // Front right wheel
    private DcMotorEx wheelBL; // Back left wheel
    private DcMotorEx wheelBR; // Back right wheel


    public void runOpMode() throws InterruptedException{

        initialize();
        moveByLine(0, 0, 0, 10, 10, 90);

    }

    public void moveByBox(double initial_x, double initial_y, double initial_angle, double target_x, double target_y, double target_angle){

        final double FORWARD_ANGLE = -90.0;

        //Handle 0 degree alignment
        double align_angle = FORWARD_ANGLE - initial_angle;
        double align_ticks = (align_angle)/360 * Math.PI * TRACK_WIDTH / WHEEL_RADIUS * TICKS; //See moveByLine for a breakdown, converts angle to ticks

        wheelFL.setTargetPosition((int) align_ticks);
        wheelFR.setTargetPosition((int) -align_ticks);
        wheelBL.setTargetPosition((int) align_ticks);
        wheelBR.setTargetPosition((int) -align_ticks);

        resetEncoders();

        //Handle movement on the vertical
        double y_ticks = (target_y - initial_y) / WHEEL_RADIUS * TICKS; //See moveByLine for a more detailed breakdown. Converts to inch distance, then to ticks.
        wheelFL.setTargetPosition((int) y_ticks);
        wheelFR.setTargetPosition((int) y_ticks);
        wheelBL.setTargetPosition((int) y_ticks);
        wheelBR.setTargetPosition((int) y_ticks);

        resetEncoders();

        //Handle movement on the horizontal
        double x_ticks = (target_x - initial_x) / WHEEL_RADIUS * TICKS; //See moveByLine for a more detailed breakdown. Converts to inch distance, then to ticks.
        wheelFL.setTargetPosition((int) x_ticks);
        wheelFR.setTargetPosition((int) -x_ticks);
        wheelBL.setTargetPosition((int) -x_ticks);
        wheelBR.setTargetPosition((int) x_ticks);

        resetEncoders();

        //Handle rotation
        double rot_ticks = target_angle/360 * Math.PI * TRACK_WIDTH / WHEEL_RADIUS * TICKS;
        wheelFL.setTargetPosition((int) rot_ticks);
        wheelFR.setTargetPosition((int) -rot_ticks);
        wheelBL.setTargetPosition((int) rot_ticks);
        wheelBR.setTargetPosition((int) -rot_ticks);

        resetEncoders();
    }

    public void moveByLine(double initial_x, double initial_y, double initial_angle, double target_x, double target_y, double target_angle){

        double x_distance = target_x - initial_x;
        double y_distance = target_y - initial_y;
        double rot_distance = (target_angle - initial_angle)/360 * Math.PI * TRACK_WIDTH; // Uses the circumference along the track width to calculate how much wheels will need to move.

        //Convert distances to ticks -- 1 rotation per wheel radius moved, TICKS ticks per rotation
        double x_ticks = x_distance / WHEEL_RADIUS * TICKS;
        double y_ticks = y_distance / WHEEL_RADIUS * TICKS;
        double rot_ticks = rot_distance / WHEEL_RADIUS * TICKS;

        wheelFL.setTargetPosition((int)(y_ticks - x_ticks - rot_ticks));
        wheelFR.setTargetPosition((int)(y_ticks + x_ticks + rot_ticks));
        wheelBL.setTargetPosition((int)(y_ticks + x_ticks - rot_ticks));
        wheelBR.setTargetPosition((int)(y_ticks - x_ticks + rot_ticks));

        resetEncoders();

    }


    //Useful stuff that help ig
    public void initialize(){
        wheelFL = hardwareMap.get(DcMotorEx.class, "wheelFL");
        wheelFR = hardwareMap.get(DcMotorEx.class, "wheelFR");
        wheelBL = hardwareMap.get(DcMotorEx.class, "wheelBL");
        wheelBR = hardwareMap.get(DcMotorEx.class, "wheelBR");

        //Reset encoder (should already be reset but whatever, redundancy!)
        wheelFL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        wheelFR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        wheelBL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        wheelBR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        wheelFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wheelFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wheelBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wheelBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        wheelFL.setDirection(DcMotorSimple.Direction.FORWARD);
        wheelFR.setDirection(DcMotorSimple.Direction.REVERSE);
        wheelBL.setDirection(DcMotorSimple.Direction.FORWARD);
        wheelBR.setDirection(DcMotorSimple.Direction.REVERSE);

        wheelFL.setPower(SPEED);
        wheelFR.setPower(SPEED);
        wheelBL.setPower(SPEED);
        wheelBR.setPower(SPEED);

    }

    public void resetEncoders(){
        wheelFL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        wheelFR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        wheelBL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        wheelBR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    }

}
//REF MAP:
/*
* |
* |
* |
* | 0 degrees (forward) <-[Bot]
* |_____________________________
*
*/