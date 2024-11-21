package org.firstinspires.ftc.teamcode.Auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Projects.HWMapBasic;


@Autonomous(name = "RRAuto")
public class RRAuto extends LinearOpMode {


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
        robot.rightLift.setTargetPosition(0);
        robot.leftLift.setTargetPosition(0);
        robot.rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        telemetry.addLine("" + robot.leftLift.getCurrentPosition());
        robot.rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        while (!isStarted()) {


        }
        waitForStart();// wait for play button to be pressed
        // autonomous happens here
       /*
        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
            Trajectory trajectory = drive.trajectoryBuilder(startPose)
                    .forward(36)
                    //lift up arm out
                    .waitSeconds(1)
                    .forward(12)
                    .waitSeconds(1)
                    //lower arm slightly
                    .backward(12)
                    //clip specimen
                    //lower arm to wall level
                    .waitSeconds(1)
                    .strafeRight(24)
                    .waitSeconds(1)
                    .forward(24)
                    .waitSeconds(1)
                    .strafeRight(24)
                    .waitSeconds(1)
                    .backward(48)
                    .waitSeconds(.25)
                    .forward(48)
                    .waitSeconds(.25)
                    .strafeRight(8)
                    .waitSeconds(.25)
                    .backward(48)
                    .waitSeconds(.25)
                    .forward(48)
                    .waitSeconds(.25)
                    .backward(48)
                    .waitSeconds(.25)
                    .forward(48)
                    .waitSeconds(.25)
                    .turn(Math.toRadians(180))
                    .waitSeconds(.1)
                    .strafeRight(24)
                    .waitSeconds(.1)
                    .forward(48)
                    //grab specimen
                    .waitSeconds(1)
                    .backward(24)
                    .turn(Math.toRadians(180))
                    .waitSeconds(.1)
                    .strafeLeft(24)
                    .waitSeconds(.1)
                    .forward(24)
                    //score specimen
                    .waitSeconds(1)
                    backward(24)
                    .waitSeconds(.1)
                    strafeRight(48)


                     .turn(Math.toDegrees(90))
                    .build();

            // Follow the trajectory
            drive.followTrajectory(trajectory);

        */


        //while(robot.right.isBusy()|| robot.left.isBusy()) {

        //telemetry.addData("Status", robot.armMotor.getCurrentPosition());
        // telemetry.update();
        //}


    }
}
