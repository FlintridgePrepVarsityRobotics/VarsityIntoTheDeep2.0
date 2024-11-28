package org.firstinspires.ftc.teamcode.Auton;

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

@Autonomous(name = "RRAuto")
public class RRAuto extends LinearOpMode {
    private SampleMecanumDrive drive;

    public HWMapBasic robot = new HWMapBasic();

    @Override
    public void runOpMode() throws InterruptedException {
        //initialize hardware map
        robot.init(hardwareMap);
        drive = new SampleMecanumDrive(hardwareMap);

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
            Trajectory trajectory = drive.trajectoryBuilder(startPose)
                    .forward(36)
                    //lift up arm out
                    .build();
                    drive.followTrajectory(trajectory);
                    sleep(1000);

        Trajectory trajectory2 = drive.trajectoryBuilder(startPose)
                    .forward(12)
                    .build();
                    drive.followTrajectory(trajectory2);
                    sleep(1000);
                    //lower arm slightly
        Trajectory trajectory3 = drive.trajectoryBuilder(startPose)
                    .back(12)
                    .build();
                    drive.followTrajectory(trajectory3);
                    sleep(1000);
                    //clip specimen
                    //lower arm to wall level
        Trajectory trajectory4 = drive.trajectoryBuilder(startPose)
                    .strafeRight(24)
                    .build();
                    drive.followTrajectory(trajectory4);
                    sleep(1000);
        Trajectory trajectory5 = drive.trajectoryBuilder(startPose)
                .forward(24)
                .build();
            drive.followTrajectory(trajectory5);
            sleep(1000);
        Trajectory trajectory6 = drive.trajectoryBuilder(startPose)
                .strafeRight(24)
                .build();
                drive.followTrajectory(trajectory6);
                sleep(1000);
        Trajectory trajectory7 = drive.trajectoryBuilder(startPose)
                .back(48)
                .build();
            drive.followTrajectory(trajectory7);
            sleep(1000);
        Trajectory trajectory8 = drive.trajectoryBuilder(startPose)
                .forward(48)
                .build();
                drive.followTrajectory(trajectory5);
                sleep(250);
        Trajectory trajectory9 = drive.trajectoryBuilder(startPose)
                .strafeRight(8)
                .build();
        drive.followTrajectory(trajectory9);
        sleep(250);
        Trajectory trajectory10 = drive.trajectoryBuilder(startPose)
                .back(48)
                .build();
                drive.followTrajectory(trajectory10);
                sleep(250);
        Trajectory trajectory11 = drive.trajectoryBuilder(startPose)
                .forward(48)
                .build();
                drive.followTrajectory(trajectory11);
                sleep(250);
        Trajectory trajectory12 = drive.trajectoryBuilder(startPose)
                .back(48)
                .build();
            drive.followTrajectory(trajectory12);
            sleep(250);
        Trajectory trajectory13 = drive.trajectoryBuilder(startPose)
                .forward(48)
                .build();
                drive.followTrajectory(trajectory13);
                sleep(250);
                //error
        TrajectorySequence trajectory14 = drive.trajectorySequenceBuilder(startPose)
                .turn(Math.toRadians(180))  // Turn 180 degrees
                .build();  // Build the trajectory sequence

        drive.followTrajectorySequence(trajectory14);
        sleep(100);  // Sleep for 100 milliseconds


        Trajectory trajectory15 = drive.trajectoryBuilder(startPose)
                .strafeRight(24)
                .build();
        drive.followTrajectory(trajectory15);
        sleep(100);
        Trajectory trajectory16 = drive.trajectoryBuilder(startPose)
                .forward(48)
                .build();
        drive.followTrajectory(trajectory16);
        sleep(1000);
        Trajectory trajectory17 = drive.trajectoryBuilder(startPose)
                .back(24)
                .build();
        drive.followTrajectory(trajectory17);
        sleep(250);
        TrajectorySequence trajectory18 = drive.trajectorySequenceBuilder(startPose)
                .turn(Math.toRadians(180))  // Turn 180 degrees
                .build();  // Build the trajectory sequence

        drive.followTrajectorySequence(trajectory18);
        sleep(100);  // Sleep for 100 milliseconds

        Trajectory trajectory19 = drive.trajectoryBuilder(startPose)
                .strafeLeft(24)
                .build();
                drive.followTrajectory(trajectory19);
                sleep(100);
        Trajectory trajectory20 = drive.trajectoryBuilder(startPose)
                .forward(24)
                .build();
                drive.followTrajectory(trajectory20);
                sleep(1000);
        Trajectory trajectory21 = drive.trajectoryBuilder(startPose)
                .back(24)
                .build();
                sleep(1000);
        Trajectory trajectory22 = drive.trajectoryBuilder(startPose)
                .strafeRight(48)
                .build();
        drive.followTrajectory(trajectory22);
        sleep(250);
        TrajectorySequence trajectory23 = drive.trajectorySequenceBuilder(startPose)
                .turn(Math.toRadians(90))  // Turn 180 degrees
                .build();  // Build the trajectory sequence

        drive.followTrajectorySequence(trajectory23);
        sleep(100);  // Sleep for 100 milliseconds





        //while(robot.right.isBusy()|| robot.left.isBusy()) {

        //telemetry.addData("Status", robot.armMotor.getCurrentPosition());
        // telemetry.update();
        //}


    }
}
