package org.firstinspires.ftc.teamcode.Auton;


import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;


import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceRunner;
import org.firstinspires.ftc.teamcode.util.LynxModuleUtil;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;


import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;


import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MOTOR_VELO_PID;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.encoderTicksToInches;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kA;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kStatic;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kV;


import org.firstinspires.ftc.teamcode.Projects.HWMapBasic;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.util.Encoder;


@Autonomous(name = "RRSpline")
public class RRSpline extends LinearOpMode {
    private SampleMecanumDrive drive;


  //  public HWMapBasic robot = new HWMapBasic();


    @Override
    public void runOpMode() throws InterruptedException {
        //initialize hardware map
//        robot.init(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
//        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        int direction = 1;
        int otherDirection = -1;
        boolean isRight = true;
        int rightPosition = 0;
        int leftPosition = 0;
        int noU = -8000;
        int[] positions;
        drive.rightLift.setTargetPosition(0);
        drive.leftLift.setTargetPosition(0);
        drive.rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        telemetry.addLine("" + drive.leftLift.getCurrentPosition());
        drive.rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drive.leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);




        while (!isStarted()) {




        }
        waitForStart();// wait for play button to be pressed
        // autonomous happens here

        //forward 20
        //forward 5
        //score
        //backward 5
        //strafe right 30
        //forward 25
        //strafe right 8
        //backward 36
        //





        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
        drive.setPoseEstimate(startPose); // Set the initial pose estimate


// Trajectory 1: Move forward 60 inches
        Trajectory trajectory1 = drive.trajectoryBuilder(startPose)
                .forward(20)
                .build();
//20, 0


// Update the pose estimate
        Pose2d currentPose = drive.getPoseEstimate();
        telemetry.addData("position", currentPose);
        telemetry.update();


        Trajectory trajectory2 = drive.trajectoryBuilder(trajectory1.end())
                .forward(8)
                .build();
//27,0
        Trajectory trajectory3 = drive.trajectoryBuilder(trajectory2.end())
                .back(8)
                .build();

        Trajectory trajectory4 = drive.trajectoryBuilder(trajectory3.end())
                .strafeRight(23)
                .build();

        Trajectory trajectory5 = drive.trajectoryBuilder(trajectory4.end())
                .splineTo(new Vector2d(53, -26), Math.toRadians(0)) //forward
                .build();

//        Trajectory trajectory6 = drive.trajectoryBuilder(trajectory5.end())
//                .strafeRight(6)                                                     //right
//                .build();

        Trajectory trajectory6 = drive.trajectoryBuilder(trajectory5.end())
                .splineToLinearHeading(new Pose2d(20, -32), Math.toRadians(0))  //back
                .build();

        Trajectory trajectory7 = drive.trajectoryBuilder(trajectory6.end())
                .splineTo(new Vector2d(40, -26), Math.toRadians(0))
                .splineTo(new Vector2d(53.5, -41.5), Math.toRadians(0))
                .build();

        Trajectory trajectory8 = drive.trajectoryBuilder(trajectory7.end())
                .splineToLinearHeading(new Pose2d(20, -43), Math.toRadians(0))  //back
                .build();

        Trajectory trajectory9 = drive.trajectoryBuilder(trajectory8.end())
                .splineTo(new Vector2d(24, -38), Math.toRadians(0))
                .build();

        Trajectory trajectory10 = drive.trajectoryBuilder(trajectory9.end())
                .back(12.3)
                .build();

        Trajectory trajectory11 = drive.trajectoryBuilder(trajectory10.end())
                .strafeLeft(40)
                .build();

        Trajectory trajectory12 = drive.trajectoryBuilder(trajectory11.end())
                .forward(24)
                .build();

        Trajectory trajectory13 = drive.trajectoryBuilder(trajectory12.end())
                .back(8)
                .build();




  //              .splineToConstantHeading(new Vector2d(50, -26), Math.toRadians(0)) //forward
  //              .splineToConstantHeading(new Vector2d(50, -36), Math.toRadians(0)) //right
   //             .splineToConstantHeading(new Vector2d(20, -36), Math.toRadians(0)) //back
  //              .splineToConstantHeading(new Vector2d(20, -28), Math.toRadians(0)) //left
//                .splineToConstantHeading(new Vector2d(49, -36), Math.toRadians(0)) //forward
  //              .splineToConstantHeading(new Vector2d(49, -44), Math.toRadians(0)) //right
//                .splineToConstantHeading(new Vector2d(20, -42), Math.toRadians(0)) //back
  //              .splineToConstantHeading(new Vector2d(33, -42), Math.toRadians(0)) //forward
//                .splineToConstantHeading(new Vector2d(50, -50), Math.toRadians(0)) //right
//                .splineToConstantHeading(new Vector2d(22, -50), Math.toRadians(0)) //back
//                .splineToConstantHeading(new Vector2d(32, -30), Math.toRadians(0)) //forward left


//                .build();


//        Trajectory trajectory4 = drive.trajectoryBuilder(trajectory3.end())
//                .back(14)
//                .build();


//        Trajectory trajectory4 = drive.trajectoryBuilder(trajectory3.end())
//                .back(27)
//                .build();


//        Trajectory trajectory5 = drive.trajectoryBuilder(trajectory3.end())
//                .splineToConstantHeading(new Vector2d(20, 0), Math.toRadians(0))
//                .build();


//        Trajectory trajectory6 = drive.trajectoryBuilder(trajectory5.end())
//                .splineToConstantHeading(new Vector2d(15,40), Math.toRadians(0))
//                .build();


        drive.followTrajectory(trajectory1); //20,0
        sleep(150);

        drive.wrist.setPosition(.35);
        drive.rightLift.setPower(.9);
        drive.leftLift.setPower(.9);
        drive.rightLift.setTargetPosition(-1680);
        drive.leftLift.setTargetPosition(-1680);
        sleep(400);

        drive.followTrajectory(trajectory2); //27,0
        sleep(100);

        drive.rightLift.setPower(.9);
        drive.leftLift.setPower(.9);
        drive.rightLift.setTargetPosition(-2500);
        drive.leftLift.setTargetPosition(-2500);
        sleep(800);

        drive.claw.setPosition(.415);

        drive.followTrajectory(trajectory3);
        sleep(100);

        drive.rightLift.setPower(.9);
        drive.leftLift.setPower(.9);
        drive.rightLift.setTargetPosition(0);
        drive.leftLift.setTargetPosition(0);
        sleep(800);

        drive.followTrajectory(trajectory4);
        sleep(280);
        drive.followTrajectory(trajectory5);
        sleep(450);
        drive.followTrajectory(trajectory6);
        sleep(620);
        drive.followTrajectory(trajectory7);
        sleep(450);
        drive.followTrajectory(trajectory8);
        sleep(200);

        drive.rArm.setPosition(.09); //arm out for spec wall grab // .07 .1 0
        drive.lArm.setPosition(.91); //.93 .9 1
        drive.wrist.setPosition(0.53);
        drive.claw.setPosition(.415);

        drive.followTrajectory(trajectory9);
        sleep(100);

        drive.followTrajectory(trajectory10); //go into wall
        sleep(300);
        drive.claw.setPosition(0);
        sleep(155);
        drive.rArm.setPosition(.9); //arm in
        drive.lArm.setPosition(.1);
        drive.wrist.setPosition(.35);

        drive.followTrajectory(trajectory11); //strafe left

        drive.rightLift.setPower(.9);
        drive.leftLift.setPower(.9);
        drive.rightLift.setTargetPosition(-1680);
        drive.leftLift.setTargetPosition(-1680);
        sleep(400);

        drive.followTrajectory(trajectory12);

        drive.rightLift.setPower(.9);
        drive.leftLift.setPower(.9);
        drive.rightLift.setTargetPosition(-2500);
        drive.leftLift.setTargetPosition(-2500);
        sleep(800);

        drive.claw.setPosition(.415);

        drive.followTrajectory(trajectory13);

        drive.rightLift.setPower(.9);
        drive.leftLift.setPower(.9);
        drive.rightLift.setTargetPosition(0);
        drive.leftLift.setTargetPosition(0);
        sleep(800);

//
//        drive.rArm.setPosition(.1);
//        drive.lArm.setPosition(.9);
//
//        drive.wrist.setPosition(0.53);
//        drive.claw.setPosition(.415);
//
//        drive.followTrajectory(trajectory11);
//
//        drive.claw.setPosition(0);
//        drive.wrist.setPosition(.35);
//        drive.rArm.setPosition(.9);
//        drive.lArm.setPosition(.1);


//        drive.rArm.setPosition(.1);
//        drive.lArm.setPosition(.9);
//
//        drive.wrist.setPosition(.65);
//        drive.claw.setPosition(.415);



//        drive.followTrajectory(trajectory4);
//        sleep(1000);
//        drive.followTrajectory(trajectory5);
//        sleep(1000);
//        drive.followTrajectory(trajectory6);






// Update the pose estimate
        while (opModeIsActive() && !isStopRequested()) {
            drive.update();  // Update odometry


            // Optionally, get and display the current pose for debugging
            Pose2d currenttPose = drive.getPoseEstimate();
            telemetry.addData("X", currenttPose.getX());
            telemetry.addData("Y", currenttPose.getY());
            telemetry.addData("Heading", currenttPose.getHeading());
            telemetry.update();
        }


        sleep(5000);












        //while(robot.right.isBusy()|| robot.left.isBusy()) {


        //telemetry.addData("Status", robot.armMotor.getCurrentPosition());
        // telemetry.update();
        //}




    }
}

