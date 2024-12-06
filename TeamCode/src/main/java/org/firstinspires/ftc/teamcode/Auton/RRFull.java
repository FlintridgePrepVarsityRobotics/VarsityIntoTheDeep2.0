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

@Autonomous(name = "RRFull")
public class RRFull extends LinearOpMode {
    private SampleMecanumDrive drive;

    public HWMapBasic robot = new HWMapBasic();

    @Override
    public void runOpMode() throws InterruptedException {
        //initialize hardware map
        robot.init(hardwareMap);
        drive = new SampleMecanumDrive(hardwareMap);
        int low = -5300;
        int high = -6880;
        int wall = -2000;

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
                .forward(27)
                .addTemporalMarker(0, () ->{
                    robot.wrist.setPosition(.53);
                    robot.rArm.setPosition(.93);
                    robot.lArm.setPosition(.07);
                    robot.rightLift.setPower(.8);
                    robot.leftLift.setPower(.8);
                    robot.rightLift.setTargetPosition(low);
                    robot.leftLift.setTargetPosition(low);
                })
                .build();
//20, 0

// Update the pose estimate
        Pose2d currentPose = drive.getPoseEstimate();
        telemetry.addData("position", currentPose);
        telemetry.update();

        Trajectory trajectory2 = drive.trajectoryBuilder(trajectory1.end())
                .forward(7)
                .build();
//27,0
        Trajectory trajectory3 = drive.trajectoryBuilder(trajectory2.end())
                .addDisplacementMarker(() -> {
                    robot.rightLift.setPower(.8);
                    robot.leftLift.setPower(.8);
                    robot.rightLift.setTargetPosition(wall);
                    robot.leftLift.setTargetPosition(wall);
                })
                .splineToConstantHeading(new Vector2d(27, -26), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(60, -26), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(60, -36), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(16, -36), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(60, -36), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(60, -46), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(20, -44), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(60, -46), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(60, -56), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(20, -56), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(32, -30), Math.toRadians(0))

                .build();

        Trajectory trajectory4 = drive.trajectoryBuilder(trajectory3.end())
                .addDisplacementMarker(() -> {
                    robot.wrist.setPosition(.53);
                    robot.rArm.setPosition(.07);
                    robot.lArm.setPosition(.93);
                })
                .back(22)
                .build();

        Trajectory trajectory5 = drive.trajectoryBuilder(trajectory4.end())
                .addDisplacementMarker(() -> {
                    robot.wrist.setPosition(.53);
                    robot.rArm.setPosition(.07);
                    robot.lArm.setPosition(.93);
                    robot.rightLift.setPower(.8);
                    robot.leftLift.setPower(.8);
                    robot.rightLift.setTargetPosition(low);
                    robot.leftLift.setTargetPosition(low);
                })
                .splineToConstantHeading(new Vector2d(20, 0), Math.toRadians(0))
                .build();

        Trajectory trajectory6 = drive.trajectoryBuilder(trajectory5.end())
                .addDisplacementMarker(() -> {
                    robot.rightLift.setPower(.8);
                    robot.leftLift.setPower(.8);
                    robot.rightLift.setTargetPosition(5);
                    robot.leftLift.setTargetPosition(5);
                })
                .splineToConstantHeading(new Vector2d(15,40), Math.toRadians(0))
                .build();

        drive.followTrajectory(trajectory1); //20,0
       // sleep(1000);
       // drive.followTrajectory(trajectory2); //27,0
        robot.rightLift.setPower(.8);
        robot.leftLift.setPower(.8);
        robot.rightLift.setTargetPosition(high);
        robot.leftLift.setTargetPosition(high);
        sleep(100);
        robot.claw.setPosition(.75);
        robot.wrist.setPosition(.825);
        sleep(100);
        drive.followTrajectory(trajectory3);
        drive.followTrajectory(trajectory4);
        sleep(100);
        robot.claw.setPosition(0);
        sleep(100);
        drive.followTrajectory(trajectory5);
        sleep(100);
        robot.rightLift.setPower(.8);
        robot.leftLift.setPower(.8);
        robot.rightLift.setTargetPosition(high);
        robot.leftLift.setTargetPosition(high);
        sleep(100);
        robot.claw.setPosition(.75);
        sleep(50);
        drive.followTrajectory(trajectory6);



// Update the pose estimate
        telemetry.addData("position", currentPose);
        telemetry.update();








        //while(robot.right.isBusy()|| robot.left.isBusy()) {

        //telemetry.addData("Status", robot.armMotor.getCurrentPosition());
        // telemetry.update();
        //}


    }
}
