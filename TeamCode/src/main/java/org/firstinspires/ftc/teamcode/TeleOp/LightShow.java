package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="Light Show", group="Linear Opmode")

public class LightShow extends LinearOpMode {

    protected DcMotor left_Back_Drive   = null;
    protected DcMotor right_Back_Drive  = null;
    protected DcMotor left_Front_Drive  = null;
    protected DcMotor right_Front_Drive = null;
    protected DcMotor left_Arm_Motor    = null;
    protected DcMotor right_Arm_Motor   = null;
    protected DcMotor pivot_Arm_Motor   = null;
    protected DcMotor ducky             = null;

    public RevBlinkinLedDriver lights;
    protected Servo basket        = null;
    private ElapsedTime runtime = new ElapsedTime();


    public void hardwareSetup()
    {
        //  Connect Motors to Phone  \\
        left_Back_Drive = hardwareMap.get(DcMotor.class, "lbD");
        right_Back_Drive = hardwareMap.get(DcMotor.class, "rbD");
        left_Front_Drive = hardwareMap.get(DcMotor.class, "lfD");
        right_Front_Drive = hardwareMap.get(DcMotor.class, "rfD");
        left_Arm_Motor = hardwareMap.get(DcMotor.class, "arm_1");
        right_Arm_Motor = hardwareMap.get(DcMotor.class, "arm_2");
        pivot_Arm_Motor = hardwareMap.get(DcMotor.class, "intake");

        basket = hardwareMap.get(Servo.class, "servo");
        lights = hardwareMap.get(RevBlinkinLedDriver.class, "lights");

//  Set the direction for each of the motors  \\
        right_Front_Drive.setDirection(DcMotorSimple.Direction.REVERSE);
        right_Back_Drive.setDirection(DcMotorSimple.Direction.REVERSE);

        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_RAINBOW_PALETTE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

    }

    //  Pause for the specified amount of time (time: mili secs)
    public void pause(double sec){
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() <= sec){

        }
    }

    @Override
    public void runOpMode() {

        hardwareSetup();
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.CONFETTI);
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("PRESENTATION MODE INITIATED :)  - ", true);
            telemetry.addData("QUESTION ASKING MODE INITIATED :D  - ", false);
            telemetry.update();

            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_OCEAN_PALETTE);
            pause(30);

            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.SINELON_OCEAN_PALETTE);
            pause(30);

            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_FOREST_PALETTE);
            pause(30);

            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_OCEAN_PALETTE);
            pause(30);

            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_FOREST_PALETTE);
            pause(30);

            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.SINELON_OCEAN_PALETTE);
            pause(30);

            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_LAVA_PALETTE);
            pause(30);

            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_FOREST_PALETTE);
            pause(30);

            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.SINELON_OCEAN_PALETTE);
            pause(30);

            telemetry.addData("PRESENTATION MODE INITIATED :)  - ", false);
            telemetry.addData("QUESTION ASKING MODE INITIATED :D  - ", true);
            telemetry.update();


            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_LAVA_PALETTE);
            pause(30);

            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.SINELON_OCEAN_PALETTE);
            pause(30);

            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_FOREST_PALETTE);
            pause(30);

            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.SINELON_LAVA_PALETTE);
            pause(30);


            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BREATH_RED);
            pause(30);

            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_BLUE);
            pause(30);

            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.SINELON_FOREST_PALETTE);
            pause(30);

            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.SINELON_OCEAN_PALETTE);
            pause(30);

            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.SINELON_FOREST_PALETTE);
            pause(30);

            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.SINELON_LAVA_PALETTE);
            pause(30);

            telemetry.addData("WE DID IT!!!  - ", true);
            telemetry.update();
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_PARTY_PALETTE);

        }


    }
}