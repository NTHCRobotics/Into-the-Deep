package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Servoooooosendhelpooooo")
public class ServoBot extends OpMode {

    private Servo servo;

    @Override
    public void init() {
        servo = hardwareMap.servo.get("servo");

        servo.setPosition(1);
        while (getRuntime()<2.0){

        }
        servo.setPosition(0);
    }

    @Override
    public void loop() {

    }
}
