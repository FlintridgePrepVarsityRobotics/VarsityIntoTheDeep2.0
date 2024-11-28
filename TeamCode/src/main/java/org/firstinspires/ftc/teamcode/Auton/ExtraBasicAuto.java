package org.firstinspires.ftc.teamcode.Auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Projects.HWMapBasic;


@Autonomous(name = "ExtraBasicAuto")
public class ExtraBasicAuto extends LinearOpMode {


    public HWMapBasic robot = new HWMapBasic();

    @Override
    public void runOpMode() throws InterruptedException {
        //initialize hardware map
        robot.init(hardwareMap);


        int direction = 1;
        int otherDirection = -1;
        boolean isRight = true;
        int rightPosition = 0;
        int leftPosition = 0;
        int noU = -8000;
        int[] positions;


        while (!isStarted()) {


        }
        waitForStart(); //wait for play button to be pressed
        // autonomous happens here
        robot.fRightWheel.setPower(.25);
        robot.fLeftWheel.setPower(.25);
        robot.bRightWheel.setPower(.25);
        robot.bLeftWheel.setPower(.25);
        sleep(750);
        robot.fRightWheel.setPower(0);
        robot.fLeftWheel.setPower(0);
        robot.bRightWheel.setPower(0);
        robot.bLeftWheel.setPower(0);
        sleep(500);


        //while(robot.right.isBusy()|| robot.left.isBusy()) {

        //telemetry.addData("Status", robot.armMotor.getCurrentPosition());
        // telemetry.update();
        //}


    }
}
