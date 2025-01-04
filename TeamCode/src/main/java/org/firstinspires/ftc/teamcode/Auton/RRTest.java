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

@Autonomous(name = "RRTest")
public class RRTest extends LinearOpMode {
    private SampleMecanumDrive drive;

    public HWMapBasic robot = new HWMapBasic();

    @Override
    public void runOpMode() throws InterruptedException {
        //initialize hardware map
       robot.init(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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
                .forward(8.98)
                .build();




//27,0

//        Trajectory trajectory3 = drive.trajectoryBuilder(trajectory2.end())
//                .strafeRight(26)
//                .lineToConstantHeading(new Vector2d(27, -26))  // right
//                .forward(27)
//                .splineToConstantHeading(new Vector2d(57, -26), Math.toRadians(0))  // forward
//                .lineToConstantHeading(new Vector2d(57, -36)) // right
//                .lineToConstantHeading(new Vector2d(16, -36))  // Back
//                .splineToConstantHeading(new Vector2d(57, -36), Math.toRadians(0))  // forward
//                .splineToConstantHeading(new Vector2d(57, -46), Math.toRadians(0))
//                .splineToConstantHeading(new Vector2d(20, -44), Math.toRadians(0))
//                .splineToConstantHeading(new Vector2d(57, -46), Math.toRadians(0))
//                .splineToConstantHeading(new Vector2d(57, -56), Math.toRadians(0))
//                .splineToConstantHeading(new Vector2d(20, -56), Math.toRadians(0))
//                .splineToConstantHeading(new Vector2d(32, -30), Math.toRadians(0))
//
//                .build();
//
//        Trajectory trajectory4 = drive.trajectoryBuilder(trajectory3.end())
//                .back(27)
//                .build();
//
//        Trajectory trajectory5 = drive.trajectoryBuilder(trajectory4.end())
//                .splineToConstantHeading(new Vector2d(20, 0), Math.toRadians(0))
//                .build();
//
//        Trajectory trajectory6 = drive.trajectoryBuilder(trajectory5.end())
//                        .splineToConstantHeading(new Vector2d(15,40), Math.toRadians(0))
//                                .build();
// -------------------
//
        Trajectory smallBackward = drive.trajectoryBuilder(trajectory2.end())
                .back(3)
                .build();
        Trajectory sideways = drive.trajectoryBuilder(smallBackward.end())
                        .strafeRight(27)
                                .build();
        Trajectory forward1 = drive.trajectoryBuilder(sideways.end())
                        .forward(29)
                                .build();
        Trajectory smallRight = drive.trajectoryBuilder(forward1.end())
                .strafeRight(13)
                                .build();
        Trajectory backward = drive.trajectoryBuilder(smallRight.end())
                        .back(48)
                                .build();
        Trajectory forwardLarge = drive.trajectoryBuilder(backward.end())
                        .forward(50)
                                .build();
        Trajectory smallRight2 = drive.trajectoryBuilder(forwardLarge.end())
                .strafeRight(8.8)
                        .build();
        Trajectory backward2 = drive.trajectoryBuilder(smallRight2.end())
                        .back(48)
                            .build();
        Trajectory forwardSmall = drive.trajectoryBuilder(backward2.end())
                .forward(12)
                .build();
        Trajectory smallLeft = drive.trajectoryBuilder(forwardSmall.end())
                .strafeLeft(23.2)
                .build();
        Trajectory Left = drive.trajectoryBuilder(trajectory1.end())
                .strafeLeft(26.8)
                .build();
        Trajectory smallBack = drive.trajectoryBuilder(smallLeft.end())
                .back(4.3)
                .build();

   //     Trajectory forward2 = drive.trajectoryBuilder(backward2.end())
   //                     .forward(48)
   //                             .build();


        // wrist midpoint :)
        robot.wrist.setPosition(.35);
        sleep(250);

        drive.followTrajectory(trajectory1); //20,0
        sleep(1000);
//
//        robot.rightLift.setPower(.8);
//     robot.leftLift.setPower(.8);
//     robot.rightLift.setTargetPosition(-2120);
//     robot.leftLift.setTargetPosition(-2120);
//     sleep(3000);

        drive.followTrajectory(trajectory2); //27,0
        sleep(500);

//        robot.rightLift.setPower(.8);
//     robot.leftLift.setPower(.8);
//     robot.rightLift.setTargetPosition(-7300);
//     robot.leftLift.setTargetPosition(-7300);
//     sleep(3300);

//     // wrist midpoint
//     robot.wrist.setPosition(.75);
//     sleep(500);

     robot.claw.setPosition(.415);
     sleep(250);

//     robot.rightLift.setPower(.8);       //lift to 0
//     robot.leftLift.setPower(.8);
//     robot.rightLift.setTargetPosition(0);
//     robot.leftLift.setTargetPosition(0);
//     sleep(4000);

     robot.claw.setPosition(0);
     sleep(250);

        drive.followTrajectory(smallBackward);
        drive.followTrajectory(sideways);
        drive.followTrajectory(forward1);
        drive.followTrajectory(smallRight);
        drive.followTrajectory(backward);

        drive.followTrajectory(forwardLarge);
        drive.followTrajectory(smallRight2);
        drive.followTrajectory(backward2);

        drive.followTrajectory(forwardSmall);
        drive.followTrajectory(smallLeft);
        robot.rArm.setPosition(.1);
        robot.lArm.setPosition(.9);
        robot.wrist.setPosition(0.5);
        robot.claw.setPosition(.415);

        drive.followTrajectory(smallBack);
        robot.claw.setPosition(0);

        robot.rArm.setPosition(.525);   // init pos
        robot.lArm.setPosition(.475);
        drive.followTrajectory(trajectory1);
        drive.followTrajectory(Left);




//        drive.followTrajectory(forward2);
//        drive.followTrajectory(sideways);
//        drive.followTrajectory(backward);

/*        drive.followTrajectory(trajectory4);
        sleep(1000);
        drive.followTrajectory(trajectory5);
        sleep(1000);
        drive.followTrajectory(trajectory6);*/



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
