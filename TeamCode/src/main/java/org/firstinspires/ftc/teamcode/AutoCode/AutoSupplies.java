package org.firstinspires.ftc.teamcode.AutoCode;
//imports

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;



public abstract class AutoSupplies extends LinearOpMode {

    //             < < < [ VARIABLES BEGIN HERE ] > > >             \\
    // Motors, servos, etc.
    protected DcMotor motorFwdLeft = null;
    protected DcMotor motorFwdRight = null;
    protected DcMotor motorBackLeft = null;
    protected DcMotor motorBackRight = null;

    protected DcMotor armOne = null;
    protected DcMotor armTwo = null;

    protected RevBlinkinLedDriver AUTOlights;

    protected BNO055IMU imu;

    protected ElapsedTime runtime = new ElapsedTime();

    //  Protected variables
    protected double globalAngle;
    protected double globalPitch;

    protected Orientation lastAngles = new Orientation();
    protected Orientation lastPitches = new Orientation();


    //Encoder Values
    //Neverest 40 motor spec: quadrature encoder, 7 pulses per revolution, count = 7 * 40
    private static final double COUNTS_PER_MOTOR_REV = 420; // Neverest 40 motor encoder - orginal val = 280
    private static final double DRIVE_GEAR_REDUCTION = 1; // This is < 1 if geared up
    private static final double COUNTS_PER_DEGREE1 = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / 360;
    //---callable methods---\\

    // Servo values \\

    protected VoltageSensor voltageSensor;

    protected Servo armServo;




    // //             < < < [ FUNCTIONS BEGIN HERE ] > > >             \\ \\

    //move
    public void encoderMove(double degrees, double x, double y) {
        resetDriveEncoders();
        double counts = degrees * COUNTS_PER_DEGREE1;
        double fwdBackPower = y;
        double strafePower = x;
        double leftFrontPower = fwdBackPower + strafePower;
        double rightFrontPower = fwdBackPower - strafePower;
        double leftBackPower = fwdBackPower - strafePower;
        double rightBackPower = fwdBackPower + strafePower;
        double maxPower;
        double max = 1.0;
        double posPower = 0.2;
        maxPower = Math.abs(leftFrontPower);
        if (Math.abs(rightFrontPower) > maxPower) {
            maxPower = Math.abs(rightFrontPower);
        }
        if (Math.abs(leftBackPower) > maxPower) {
            maxPower = Math.abs(leftBackPower);
        }
        if (Math.abs(rightBackPower) > maxPower) {
            maxPower = Math.abs(rightBackPower);
        }
        if (maxPower > 1) {
            leftFrontPower = leftFrontPower / maxPower;
            rightFrontPower = rightFrontPower / maxPower;
            leftBackPower = leftBackPower / maxPower;
            rightBackPower = rightBackPower / maxPower;

        }
        //sets the power of the motors
        double averageEnc = (Math.abs(motorFwdLeft.getCurrentPosition())
                + Math.abs(motorFwdRight.getCurrentPosition())
                + Math.abs(motorBackLeft.getCurrentPosition())
                + Math.abs(motorBackRight.getCurrentPosition())) / 4.0;
        while (opModeIsActive() && averageEnc <= counts) {
            averageEnc = (Math.abs(motorFwdLeft.getCurrentPosition())
                    + Math.abs(motorFwdRight.getCurrentPosition())
                    + Math.abs(motorBackLeft.getCurrentPosition())
                    + Math.abs(motorBackRight.getCurrentPosition())) / 4.0;
            if (posPower < 1 && averageEnc / counts < .6) {
                posPower *= 1.1;
            } else if (posPower >= 1 && averageEnc / counts < .6) {
                posPower = 1;
            } else if (averageEnc / counts >= .6 && posPower >= .25) {
                posPower *= .99;
            } else {
                posPower = .25;
            }
            telemetry.addData("max :: ", max);
            telemetry.addData("posPower :: ", posPower);
            telemetry.update();
            motorFwdLeft.setPower(leftFrontPower * max * posPower);
            motorBackLeft.setPower(leftBackPower * max * posPower);
            motorFwdRight.setPower(rightFrontPower * max * posPower);
            motorBackRight.setPower(rightBackPower * max * posPower);
        }
        motorFwdLeft.setPower(0);
        motorBackLeft.setPower(0);
        motorFwdRight.setPower(0);
        motorBackRight.setPower(0);
    }

    public void resetDriveEncoders() {
        motorFwdLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFwdLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFwdRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFwdRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


    public void resetArmEncoders() {
        // Reset all encoders and set their modes \\
        armOne.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armTwo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armOne.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armTwo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void ArmUPbottom(double power) {
        telemetry.addData("armOne Encoder Val :: ", armOne.getCurrentPosition());
        telemetry.update();
        armOne.setPower(power);
        armOne.setTargetPosition(1392);
        armOne.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void ArmEXTRUDEbottom(double power) {
        telemetry.addData("armTwo Encoder Val :: ", armTwo.getCurrentPosition());
        telemetry.update();
        armTwo.setPower(power);
        armTwo.setTargetPosition(2930);
        armTwo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }


    public void ArmBACKin(double power, double volts) {
        armTwo.setPower(power);
        armTwo.setTargetPosition(0);
        if (volts >= 11.0 && armTwo.getCurrentPosition() >= 24) {
            telemetry.addData("ArmTwo Pos = ", armTwo.getCurrentPosition());
            telemetry.update();
            armTwo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            AUTOlights.setPattern(RevBlinkinLedDriver.BlinkinPattern.CONFETTI);
            volts = getVoltage();
            ArmBACKin(power, volts);
            AUTOlights.setPattern(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_RED);
        } else {
            AUTOlights.setPattern(RevBlinkinLedDriver.BlinkinPattern.TWINKLES_LAVA_PALETTE);
            armTwo.setPower(0);
            armTwo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            armTwo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        }
    }

    public double getVoltage() {
        voltageSensor = hardwareMap.voltageSensor.iterator().next();
        return voltageSensor.getVoltage();
    }

    public void ArmDOWNin(double power, double volts) {
        armOne.setPower(power);
        armOne.setTargetPosition(0);
        if (volts >= 11.0 && armOne.getCurrentPosition() >= 24) {
            telemetry.addData("ArmOne Pos = ", armOne.getCurrentPosition());
            telemetry.update();
            armOne.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            AUTOlights.setPattern(RevBlinkinLedDriver.BlinkinPattern.CONFETTI);
            volts = getVoltage();
            ArmDOWNin(power, volts);
            AUTOlights.setPattern(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_RED);

        } else {
            AUTOlights.setPattern(RevBlinkinLedDriver.BlinkinPattern.TWINKLES_LAVA_PALETTE);
            armOne.setPower(0);
            armOne.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            armOne.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        }
    }

    public void dropPixel(double openVal) {
        armServo.setPosition(openVal);
    }

    public void closePixel(double closeVal) {
        armServo.setPosition(closeVal);
    }

    //Using the gyroscope, when a degree is passed both left and right motors move accordingly in
    //order to turn the robot to the right or left until the bearing is equal to or greater than the
    //specified degree. Power can also be specified.
    //used commonly in pairs(one fast for speed and one slow for accuracy) to improve movement time.
    public void turnToS(int degrees, double power, int loopnum) {
        int left = 1;
        int right = 1;
        double distance = getAngle() - degrees;
        double startAngle = getAngle();
        telemetry.addData("Angle3", getAngle());
        telemetry.update();
        if (getAngle() <= degrees) {
            left *= -1;
        } else if (getAngle() > degrees) {
            right *= -1;
        }

        motorBackLeft.setPower(power * left);
        motorFwdLeft.setPower(power * left);
        motorBackRight.setPower(power * right);
        motorFwdRight.setPower(power * right);

        if (getAngle() > degrees) {
            // On left turn we have to get off zero first.
            while (opModeIsActive() && getAngle() >= degrees) {
                //telemetry.addData("Angle4",getAngle());
                //telemetry.update();
                if ((startAngle + ((distance / 4) * 3)) > getAngle()) {
                    left *= 1.05;
                    right *= 1.05;
                } else {
                    if (left > 1 || left < -1 || right > 1 || right < -1) {
                        left *= 0.95;
                        right *= 0.95;
                    }
                }
            }
        } else {    // right turn.
            while (opModeIsActive() && getAngle() <= degrees) {
                //telemetry.addData("Angle4", getAngle());
                //telemetry.update();
                if ((startAngle + ((distance / 4) * 3)) > getAngle()) {
                    left *= 1.05;
                    right *= 1.05;
                } else {
                    if (left > 1 || left < -1 || right > 1 || right < -1) {
                        left *= 0.95;
                        right *= 0.95;
                    }
                }
            }
        }
        // turn the motors off.
        motorBackLeft.setPower(0);
        motorFwdLeft.setPower(0);
        motorBackRight.setPower(0);
        motorFwdRight.setPower(0);
        if (--loopnum > 0) {
            turnToS(degrees, power / 2, loopnum);
        }
    }



    //  Pause for the specified amount of time (time: mili secs)
    public void pause(long millis) {
        runtime.reset();
        while (opModeIsActive() && runtime.milliseconds() <= millis) {

        }
    }

    //Resets gyro sensor bearing value to 0
    //commonly used to calibrate before a match as well
    public void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    //Resets gyro sensor pitch value to 0
    //commonly used to calibrate before a match as well
    public void resetPitch() {
        lastPitches = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalPitch = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     *
     * @return Angle in degrees. + = left, - = right.
     */
    //uses the imu to find the current angle
    public double getAngle() {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    //uses the imu to get the current pitch of the robot
    public double getPitch() {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.secondAngle - lastPitches.secondAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalPitch += deltaAngle;

        lastPitches = angles;

        return globalPitch;
    }


    public void initForAutonomous() {
        //Prepares all the hardware OUTSIDE OF ODOMETRY

        armOne = hardwareMap.get(DcMotor.class, "arm_1");
        armOne.setDirection(DcMotor.Direction.REVERSE);
        armTwo = hardwareMap.get(DcMotor.class, "arm_2");
        armTwo.setDirection(DcMotor.Direction.REVERSE);

        resetArmEncoders();


        armServo = hardwareMap.get(Servo.class, "servo");


        AUTOlights = hardwareMap.get(RevBlinkinLedDriver.class, "lights");

    }

    public void initLanaAuto() {
        //Prepares all the hardware
        motorFwdRight = hardwareMap.get(DcMotor.class, "motorFwdRight");
        motorBackLeft = hardwareMap.get(DcMotor.class, "motorBackLeft");
        motorFwdLeft = hardwareMap.get(DcMotor.class, "motorFwdLeft");
        motorBackRight = hardwareMap.get(DcMotor.class, "motorBackRight");
        motorBackLeft.setDirection(DcMotor.Direction.REVERSE);
        //motorBackRight.setDirection(DcMotor.Direction.REVERSE);
        //motorFwdRight.setDirection(DcMotor.Direction.REVERSE);


        motorFwdRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFwdLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;


        AUTOlights = hardwareMap.get(RevBlinkinLedDriver.class, "lights");

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        BNO055IMU.Parameters gyroParameters = new BNO055IMU.Parameters();

        gyroParameters.mode = BNO055IMU.SensorMode.IMU;
        gyroParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        gyroParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        gyroParameters.loggingEnabled = false;

        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFwdRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFwdLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        //initializes imu and calibrates it. Prepares lift motor to land using the encoder
        // Lights turn green when it is calibrated
        telemetry.addData("Mode", "calibrating...");
        telemetry.update();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(gyroParameters);

        telemetry.clear();
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

}
