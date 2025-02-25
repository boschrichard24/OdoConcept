package org.firstinspires.ftc.teamcode.AutoCode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;

@Autonomous(name="TestOne", group="CompetitionAuto")
public class TestOne extends AutoSupplies {

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
            ArmUPbottom(1);
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .splineTo(new Vector2d(30, 30), Math.PI / 2)
                            .splineTo(new Vector2d(60, 0), Math.PI)
                            .build());

            ArmEXTRUDEbottom(1.5);
            pause(4000);
            ArmBACKin(-1.5,this.voltageSensor.getVoltage());
            pause(2000);
            ArmDOWNin(-1, this.voltageSensor.getVoltage());
            pause(4000);



        }
            else {
                throw new AssertionError();
            }
        }
}
