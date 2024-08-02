package org.firstinspires.ftc.teamcode.TeleOp;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

//import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

@TeleOp(name="SimpleDrive", group="Linear Opmode")
//@Disabled

public class SimpleDrive extends LinearOpMode {

    private ElapsedTime runtime       = new ElapsedTime();
    private DcMotor left_Back_Drive   = null;
    private DcMotor right_Back_Drive  = null;
    private DcMotor left_Front_Drive  = null;
    private DcMotor right_Front_Drive = null;
    private DcMotor arm_1 = null;
    private DcMotor arm_2 = null;
    private DcMotor Intake   = null;

    Servo armServo;

    RevBlinkinLedDriver lights;

    /* Declare OpMode members. */
    /*
    HardwarePushbot robot           = new HardwarePushbot();   // Use a Pushbot's hardware
    double          clawOffset      = 0;                       // Servo mid position
    final double    CLAW_SPEED      = 0.02 ;                   // sets rate to move servo
*/
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        left_Back_Drive   = hardwareMap.get(DcMotor.class, "lbD");
        right_Back_Drive  = hardwareMap.get(DcMotor.class, "rbD");
        left_Front_Drive  = hardwareMap.get(DcMotor.class, "lfD");
        right_Front_Drive = hardwareMap.get(DcMotor.class, "rfD");
        arm_1 = hardwareMap.get(DcMotor.class, "arm_1");
        arm_2 = hardwareMap.get(DcMotor.class, "arm_2");

        armServo = hardwareMap.get(Servo.class, "servo");

        lights = hardwareMap.get(RevBlinkinLedDriver.class, "lights");

        Intake   = hardwareMap.get(DcMotor.class, "intake");

        // Set the direction for each of the motors \\
        right_Front_Drive.setDirection(DcMotorSimple.Direction.REVERSE);
        right_Back_Drive.setDirection(DcMotorSimple.Direction.REVERSE);

        Intake.setDirection(DcMotorSimple.Direction.FORWARD);
        arm_2.setDirection(DcMotorSimple.Direction.REVERSE);
        arm_1.setDirection(DcMotorSimple.Direction.REVERSE);

        // Wait for the start button
        telemetry.addData(">", "Ready to run program.");
        telemetry.update();

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();

        // Resetting Encoder stuff
        arm_1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm_2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_Back_Drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_Front_Drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_Back_Drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_Front_Drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // ---------------------------------------------------------------- \\
        //Setting Encoders
        arm_1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm_2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        Intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        //Power
        double leftPower;
        double rightPower;
        double armLeftPower;
        double armRightPower;
        double spinPower;
        double duckyPower = 0.8;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Tank Mode uses one stick to control each wheel.
            leftPower  = -gamepad1.left_stick_y;
            rightPower = -gamepad1.right_stick_y;

            //DuckyCode                             P R O B L E M  !!!

            /*if(gamepad1.left_bumper){
                double freeze = runtime.seconds();
                while(freeze > runtime.seconds()) {
                    ducky.setPower(duckyPower);
                    duckyPower += 0.02;
                }
            }*/

            //Arm code
            double armDriveLeft  = -gamepad2.left_stick_y;
            double armDriveRight = -gamepad2.right_stick_y;
            armLeftPower  = Range.clip(armDriveLeft, -0.08, 0.08);
            armRightPower = Range.clip(armDriveRight, -0.08, 0.08);

            //Spin Code
            double spinLeft = gamepad2.left_stick_x;
            spinPower       = Range.clip(spinLeft, -0.3, 0.3);

            // Send calculated power to wheels
            left_Back_Drive.setPower(leftPower);
            right_Back_Drive.setPower(rightPower);
            left_Front_Drive.setPower(leftPower);
            right_Front_Drive.setPower(rightPower);
            arm_1.setPower(armLeftPower);
            arm_2.setPower(armRightPower);
            Intake.setPower(spinPower);

            // Use gamepad left & right Bumpers to open and close the claw
            /*if (gamepad2.dpad_up)
                clawOffset += CLAW_SPEED;
            else if (gamepad2.dpad_down)
                clawOffset -= CLAW_SPEED;

            // Move both servos to new position.  Assume servos are mirror image of each other.
            clawOffset = Range.clip(clawOffset, -0.5, 0.5);
            robot.leftClaw.setPosition(robot.MID_SERVO + clawOffset);
            robot.rightClaw.setPosition(robot.MID_SERVO - clawOffset);


            // Send telemetry message to signify robot running;
            telemetry.addData("claw",  "Offset = %.2f", clawOffset);
            telemetry.update();
*/
            // Pace this loop so jaw action is reasonable speed.

            // Show the elapsed game time and wheel power
            telemetry.addData("Swivel Left Power:", armLeftPower);
            telemetry.addData("Swivel Right Power:", armRightPower);

            //get updated encoder positions
            telemetry.addData("Arm One Value:",  arm_1.getCurrentPosition());
            telemetry.addData("Arm Two Value:",  arm_2.getCurrentPosition());
            /*
            telemetry.addData("DriveLeft", "front (%10d), back (%10d)",
                    left_Front_Drive.getCurrentPosition(),
                    left_Back_Drive.getCurrentPosition());
            telemetry.addData("DriveRight", "front (%10d), back (%10d)",
                    right_Front_Drive.getCurrentPosition(),
                    right_Back_Drive.getCurrentPosition());
*/
            // Display the current value
            telemetry.addData(">", "Press Stop to end test." );
            telemetry.update();
        }

        // Signal done;
        //telemetry.addData("Trigger_Value", triggerValue);         Commented out on 11/23/21
        telemetry.addData(">", "Done");
        telemetry.update();
    }
}