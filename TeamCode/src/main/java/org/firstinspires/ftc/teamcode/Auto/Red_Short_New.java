package org.firstinspires.ftc.teamcode.Auto;

//Imports (Literally just Austin's stuff)
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

//NOTE: All units in inches/feet! (We're American here)
public class Red_Short_New extends LinearOpMode{

    //Declare runtimer
    private ElapsedTime runtime = new ElapsedTime();

    //Declare/set motors
    private DcMotorEx wheelFL = hardwareMap.get(DcMotorEx.class, "wheelFL");
    private DcMotorEx wheelFR;
    private DcMotorEx wheelBL;
    private DcMotorEx wheelBR;
    //List of all motors for ease-of-access


    //TODO: Declare viper slide extension, claw, claw arm rotation

    //Declare/set constant variables
    static final double VSAngle = 80.0; //TODO: REPLACE WITH ACTUAL ANGLE!
    static final double VSLength = 53.543; //From GoBILDA website, total length of fully extended slide
    static final double ENCRATE = 537.689839572; //From GoBilda website, Yellow Jacket 5303
    //distPerRot gives the distance the bot will travel forwards for each rotation of the wheel. Works for forward/backward movement
    //If fine movement is needed, use the IMU
    static final double distPerRot = 2.0 * Math.PI * 1.89; // c=(2)(pi)(r), r = 3.78in/2 = 1.89in
    static final double height = 2.0; //Height of viper slide above the ground
    static final double initialX = 0.0; //Initial x position, there will be no variance in y, therefore no variable. Maybe change later :p
    static final double SPEED = 1.0;

    @Override
    public void runOpMode() throws InterruptedException{
        waitForStart();
        telemetry.addData("Status", "Ready to run");
        waitForStart();
        double target = 60.0 - VSLength * Math.cos(VSAngle); //Calculates distance from bucket needed to extend and hit
        move(initialX, target);
    }

    public void move(double x, double targetX){
        //
        setModeToAllWheels(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setModeToAllWheels(DcMotor.RunMode.RUN_TO_POSITION);

        double targetPosition = targetX - x; //Gets the distance between current and target positions

        double ticksToTarget = Math.abs(targetPosition) / distPerRot * ENCRATE;

        setTargetToAllWheels(ticksToTarget);

        if(targetPosition < 0){
            setPowerToAllWheels(-SPEED);
        }
        else if(targetPosition > 0) {
            setPowerToAllWheels(SPEED);
        }

        while(opModeIsActive() && wheelFL.isBusy()){
            telemetry.addData("Target", ticksToTarget);
            telemetry.addData("Current", wheelFL.getCurrentPosition());
        }
        setPowerToAllWheels(0);
        //
    }

    public void setModeToAllWheels(DcMotor.RunMode mode){
        wheelFL.setMode(mode);
        wheelFR.setMode(mode);
        wheelBL.setMode(mode);
        wheelFR.setMode(mode);
    }
    public void setTargetToAllWheels(double target){
        wheelFL.setTargetPosition((int) Math.round(target));
        wheelFR.setTargetPosition((int) Math.round(target));
        wheelBL.setTargetPosition((int) Math.round(target));
        wheelFR.setTargetPosition((int) Math.round(target));
    }
    public void setPowerToAllWheels(double power){
        wheelFL.setPower(power);
        wheelFR.setPower(power);
        wheelBL.setPower(power);
        wheelFR.setPower(power);
    }
}
