package org.firstinspires.ftc.teamcode.AutoCode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;

@Autonomous(name="TestTwo", group="CompetitionAuto")
public class TestTwo extends AutoSupplies {

    public VoltageSensor voltageSensor;

    @Override
    public void runOpMode() throws InterruptedException {


        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
            voltageSensor = hardwareMap.voltageSensor.iterator().next();
            initForAutonomous();
            waitForStart();


            AUTOlights = hardwareMap.get(RevBlinkinLedDriver.class, "lights");
            AUTOlights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BREATH_BLUE);
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .splineTo(new Vector2d(48, 24), Math.PI / 2)
                            .splineTo(new Vector2d(48, 24), Math.PI / 2)
                            .splineTo(new Vector2d(48, 40), Math.PI / 2)
                            .splineTo(new Vector2d(24, 66), Math.PI / 2)
                            .build());
            ArmUPbottom(1);
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .lineToY(75)
                            .build());

            ArmEXTRUDEbottom(1.5);
            pause(1000);
            dropPixel(0.259);
            pause(1000);
            closePixel(-4.149);
            pause(1000);
            dropPixel(0.259);
            pause(1000);
            closePixel(-4.149);
            pause(1000);
            ArmBACKin(-1.5,this.voltageSensor.getVoltage());
            pause(2000);
            ArmDOWNin(-1, this.voltageSensor.getVoltage());
            pause(2000);



        }
            else {
                throw new AssertionError();
            }
        }
}
