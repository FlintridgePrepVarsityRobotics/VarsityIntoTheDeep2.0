package org.firstinspires.ftc.teamcode.Auton;
import com.acmerobotics.roadrunner.geometry.Pose2d;
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

@Autonomous(name = "OdometryAuto")
public class OdometryAuto extends LinearOpMode {
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
        robot.wrist.setPosition(.53);

        robot.rArm.setPosition(.93);
        robot.lArm.setPosition(.07);
        sleep(500);

        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
        drive.setPoseEstimate(startPose); // Set the initial pose estimate

        Trajectory trajectory1 = drive.trajectoryBuilder(startPose)     //drive forward 1 tile
                .forward(18)
                .build();
        drive.followTrajectory(trajectory1);
        sleep(500);

    /*    robot.rightLift.setPower(.8);       //lift up above first spec bar
        robot.leftLift.setPower(.8);
        robot.rightLift.setTargetPosition(-1600);//-850
        robot.leftLift.setTargetPosition(-1600);//-850
        sleep(1000);*/

        // wrist midpoint :)
   /*     robot.wrist.setPosition(.66); //.65, .73
        sleep(250);*/

        robot.rightLift.setPower(.8);       //lift to high spec scoring height
        robot.leftLift.setPower(.8);
        robot.rightLift.setTargetPosition(-5300);//-2020
        robot.leftLift.setTargetPosition(-5300);//-2020
        sleep(3300);

        Pose2d startPose2 = new Pose2d(18, 0, Math.toRadians(0));       //drive forward to submersible structure
        drive.setPoseEstimate(startPose2);
        Trajectory trajectory2 = drive.trajectoryBuilder(startPose2)
                .forward(7.6)
                .build();
        drive.followTrajectory(trajectory2);
        sleep(1000);

        robot.rightLift.setPower(.8);       //lift to high spec scoring height
        robot.leftLift.setPower(.8);
        robot.rightLift.setTargetPosition(-6880);//-2020
        robot.leftLift.setTargetPosition(-6880);//-2020
        sleep(3300);
/*        robot.wrist.setPosition(.7); //.768
        sleep(250);*/

       /* Pose2d startPose3 = new Pose2d(22, 0, Math.toRadians(0));       //drive backward from submersible structure
        drive.setPoseEstimate(startPose3);
        Trajectory trajectory3 = drive.trajectoryBuilder(startPose3)
                .forward(-5)
                .build();
        drive.followTrajectory(trajectory3);
        sleep(250);*/

        robot.claw.setPosition(.415);
        sleep(250);

        robot.rightLift.setPower(.8);       //lift to 0
        robot.leftLift.setPower(.8);
        robot.rightLift.setTargetPosition(0);
        robot.leftLift.setTargetPosition(0);
        sleep(3800);

        drive.update(); // Updates the localizer within SampleMecanumDrive


        // Print position to telemetry
        telemetry.addData("position", drive.getPoseEstimate());
        telemetry.update();

    }
}

