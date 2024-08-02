package org.firstinspires.ftc.teamcode.AutoCode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

@Autonomous(name="Draw Lana's Name", group="CompetitionAuto")
public class LanaCode extends AutoSupplies{

    public void runOpMode() {
        //initLanaAuto();

        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        //Start programming

        //L
        AUTOlights.setPattern(RevBlinkinLedDriver.BlinkinPattern.TWINKLES_FOREST_PALETTE);
        encoderMove(200,0,1);
        pause(2000);
        turnToS(-90, 0.5, 3);
        pause(2000);
        encoderMove(100,0,1);
        pause(2000);

        //A
        AUTOlights.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_FOREST_PALETTE);
        encoderMove(280,-1,1);
        pause(2000);
        encoderMove(140,1,1);
        pause(2000);
        turnToS(180, 0.5, 3);
        pause(2000);
        encoderMove(150,0,1);
        pause(2000);
        turnToS(180, 0.5, 3);
        pause(2000);
        encoderMove(150,0,1);
        pause(2000);
        encoderMove(140,1,1);
        pause(2000);

        //N
        AUTOlights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_FOREST_PALETTE);
        turnToS(-90, 0.5, 3);
        pause(2000);
        encoderMove(200,0,1);
        pause(2000);
        encoderMove(280,1,-1);
        pause(2000);
        encoderMove(200,0,1);
        pause(2000);
        encoderMove(200,0,-1);
        pause(2000);
        turnToS(90, 0.5, 3);
        pause(2000);

        //A
        AUTOlights.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_FOREST_PALETTE);
        encoderMove(280,-1,1);
        pause(2000);
        encoderMove(140,1,1);
        pause(2000);
        turnToS(180, 0.5, 3);
        pause(2000);
        encoderMove(150,0,1);
        pause(2000);
        turnToS(180, 0.5, 3);
        pause(2000);
        encoderMove(150,0,1);
        pause(2000);
        encoderMove(140,1,1);
        pause(2000);

        //Celebrate :D
        AUTOlights.setPattern(RevBlinkinLedDriver.BlinkinPattern.CONFETTI);
        turnToS(720, 1, 2);


    }
}
