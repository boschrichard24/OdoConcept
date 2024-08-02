package org.firstinspires.ftc.teamcode.TeleOp;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "ServoTest", group = "TeleOp")

public class ServoTest extends LinearOpMode {
    //All hardware
    protected DcMotor left_Back_Drive   = null;
    protected DcMotor right_Back_Drive  = null;
    protected DcMotor left_Front_Drive  = null;
    protected DcMotor right_Front_Drive = null;
    protected DcMotor left_Arm_Motor    = null;
    protected DcMotor right_Arm_Motor   = null;
    protected DcMotor pivot_Arm_Motor   = null;
    protected DcMotor ducky             = null;
    protected Servo claw        = null;  // This is the open and close servo of the claw \\

    double servo = 0.5;

    @Override
    public void runOpMode() {
        //Prepares all the hardware
        left_Back_Drive = hardwareMap.get(DcMotor.class, "lbD");
        right_Back_Drive = hardwareMap.get(DcMotor.class, "rbD");
        left_Front_Drive = hardwareMap.get(DcMotor.class, "lfD");
        right_Front_Drive = hardwareMap.get(DcMotor.class, "rfD");
        left_Arm_Motor = hardwareMap.get(DcMotor.class, "arm_1");
        right_Arm_Motor = hardwareMap.get(DcMotor.class, "arm_2");
        pivot_Arm_Motor = hardwareMap.get(DcMotor.class, "intake");
        claw = hardwareMap.get(Servo.class, "servo");

        right_Front_Drive.setDirection(DcMotorSimple.Direction.REVERSE);
        right_Back_Drive.setDirection(DcMotorSimple.Direction.REVERSE);


        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            if(gamepad1.dpad_up){
                servo += 0.001;
            }
            else if(gamepad1.dpad_down){
                servo -= 0.001;
            }

            claw.setPosition(servo);

            telemetry.addData("servo", servo);
            telemetry.addData("claw", claw);
            telemetry.update();
        }
    }
}