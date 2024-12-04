package org.firstinspires.ftc.teamcode.Auton;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
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

@Autonomous(name = "FullRRAuto")
public class FullRRAuto extends LinearOpMode {
    private SampleMecanumDrive drive;

    public HWMapBasic robot = new HWMapBasic();

    @Override
    public void runOpMode() throws InterruptedException {
        //initialize hardware map
        robot.init(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

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
        robot.wrist.setPosition(.53);

        robot.rArm.setPosition(.93);
        robot.lArm.setPosition(.07);
        sleep(500);

        Pose2d startPose = new Pose2d(0, 10, Math.toRadians(0));
        drive.setPoseEstimate(startPose); // Set the initial pose estimate

        Trajectory trajectory1 = drive.trajectoryBuilder(startPose)     //drive forward 1 tile
                .splineTo(new Vector2d(18, 0), Math.toRadians(0)) //forward
                .build();
        drive.followTrajectory(trajectory1);
        sleep(500);



        robot.rightLift.setPower(.8);       //lift to high spec scoring height
        robot.leftLift.setPower(.8);
        robot.rightLift.setTargetPosition(-5400);//-2020
        robot.leftLift.setTargetPosition(-5400);//-2020
        sleep(3300);

        Pose2d startPose2 = new Pose2d(18, 0, Math.toRadians(0));       //drive forward to submersible structure
        drive.setPoseEstimate(startPose2);
        Trajectory trajectory2 = drive.trajectoryBuilder(startPose2)
                .forward(7.8)
                .build();
        drive.followTrajectory(trajectory2);
        sleep(1000);

        robot.rightLift.setPower(.8);       //lift to high spec scoring height
        robot.leftLift.setPower(.8);
        robot.rightLift.setTargetPosition(-6880);//-2020
        robot.leftLift.setTargetPosition(-6880);//-2020
        sleep(3300);

        robot.claw.setPosition(.415);
        sleep(250);

        robot.rightLift.setPower(.8);       //lift to 0
        robot.leftLift.setPower(.8);
        robot.rightLift.setTargetPosition(10);
        robot.leftLift.setTargetPosition(10);
        sleep(3800);

        Pose2d startPose3 = new Pose2d(25.8, 0, Math.toRadians(0));       //drive forward to submersible structure
        drive.setPoseEstimate(startPose3);
        Trajectory trajectory3 = drive.trajectoryBuilder(startPose2)
                .back(5.8)
                .build();
        drive.followTrajectory(trajectory3);
        sleep(500);

        Pose2d startPose4 = new Pose2d(20, 0, Math.toRadians(0));       //drive forward to submersible structure
        drive.setPoseEstimate(startPose4);
        Trajectory trajectory4 = drive.trajectoryBuilder(startPose2)
                .strafeRight(24)
                .build();
        drive.followTrajectory(trajectory3);
        sleep(500);

        Pose2d startPose5 = new Pose2d(20, 24, Math.toRadians(0));       //drive forward to submersible structure
        drive.setPoseEstimate(startPose5);
        Trajectory trajectory5 = drive.trajectoryBuilder(startPose2)
                .splineTo(new Vector2d(50, 24), Math.toRadians(0)) //forward
                .splineToConstantHeading(new Vector2d(50,36 ), Math.toRadians(0)) //strafe
                .splineTo(new Vector2d(5, 36), Math.toRadians(0)) //back
                .splineTo(new Vector2d(50, 36), Math.toRadians(0)) //forward
                .splineToConstantHeading(new Vector2d(50,43 ), Math.toRadians(0)) //strafe
                .splineTo(new Vector2d(5, 43), Math.toRadians(0)) //back
                .splineTo(new Vector2d(50, 43), Math.toRadians(0)) //forward
                .splineToConstantHeading(new Vector2d(50,50 ), Math.toRadians(0)) //strafe
                .splineTo(new Vector2d(5, 59), Math.toRadians(0)) //back
                .splineTo(new Vector2d(40, 38), Math.toRadians(0)) //forward

                .build();
        drive.followTrajectory(trajectory5);
        sleep(500);
        drive.turn(Math.toRadians(180));

        Pose2d startPose6 = new Pose2d(40, 38, Math.toRadians(0));       //drive forward to submersible structure
        drive.setPoseEstimate(startPose6);
        Trajectory trajectory6 = drive.trajectoryBuilder(startPose2)
                .splineTo(new Vector2d(3, 38), Math.toRadians(0)) //forward
                .build();
        drive.followTrajectory(trajectory6);
        sleep(500);

        //lift and claw
        robot.claw.setPosition(.73);
        robot.rightLift.setPower(.8);       //lift to 0
        robot.leftLift.setPower(.8);
        robot.rightLift.setTargetPosition(-1000);
        robot.leftLift.setTargetPosition(-1000);
        sleep(1000);
        robot.claw.setPosition(.23);

        Pose2d startPose7 = new Pose2d(3, 38, Math.toRadians(0));       //drive forward to submersible structure
        drive.setPoseEstimate(startPose7);
        Trajectory trajectory7 = drive.trajectoryBuilder(startPose2)
                .splineTo(new Vector2d(10, 38), Math.toRadians(0)) //back
                .splineTo(new Vector2d(18, 0), Math.toRadians(0))
                .build();
        drive.followTrajectory(trajectory7);
        sleep(500);
        drive.turn(Math.toRadians(180));
        //lift
        robot.rightLift.setPower(.8);       //lift to high spec scoring height
        robot.leftLift.setPower(.8);
        robot.rightLift.setTargetPosition(-700);//-2020
        robot.leftLift.setTargetPosition(-7000);//-2020
        sleep(3300);
        drive.followTrajectory(trajectory2);
        //lift
        robot.rightLift.setPower(.8);       //lift to high spec scoring height
        robot.leftLift.setPower(.8);
        robot.rightLift.setTargetPosition(-6600);//-2020
        robot.leftLift.setTargetPosition(-6600);//-2020
        sleep(3300);

        robot.claw.setPosition(.415);
        sleep(250);

        robot.rightLift.setPower(.8);       //lift to 0
        robot.leftLift.setPower(.8);
        robot.rightLift.setTargetPosition(10);
        robot.leftLift.setTargetPosition(10);
        sleep(3800);
        drive.followTrajectory(trajectory3);

        Pose2d startPose8 = new Pose2d(20, 0, Math.toRadians(0));       //drive forward to submersible structure
        drive.setPoseEstimate(startPose8);
        Trajectory trajectory8 = drive.trajectoryBuilder(startPose2)
                .lineTo(new Vector2d(5, 48))
                .build();
        drive.followTrajectory(trajectory8);
        sleep(500);



        drive.update(); // Updates the localizer within SampleMecanumDrive


        // Print position to telemetry
        telemetry.addData("position", drive.getPoseEstimate());
        telemetry.update();

    }
}

