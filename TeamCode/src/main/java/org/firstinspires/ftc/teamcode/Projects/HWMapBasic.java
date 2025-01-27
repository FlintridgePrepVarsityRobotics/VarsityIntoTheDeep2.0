package org.firstinspires.ftc.teamcode.Projects;

import android.widget.Button;
import com.google.blocks.ftcrobotcontroller.util.ProjectsUtil;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Projects.Project;

public class HWMapBasic extends Project {
    public DcMotor fLeftWheel = null;  //control hub port 2
    public DcMotor fRightWheel = null; //control hub port 3
    public DcMotor bLeftWheel = null; //control hub port 1
    public DcMotor bRightWheel = null; //control hub port 0
    public DcMotorEx leftLift = null; // expan hub motor 1
    public DcMotorEx rightLift = null; // expan hub motor 0
    public Servo claw = null; // control hub servo 2
    public Servo wrist = null; // expan hub servo 3

    public Servo lBar = null; // control hub servo 1
    public Servo rBar = null; // expan hub servo 1
    public Servo lArm = null; // control hub servo 0
    public Servo rArm = null; // expan hub servo 2
    public Servo rDiff = null;
    public Servo lDiff = null;
    public WebcamName camera = null;

    //@Override
    public void init(HardwareMap hwMap) {
        // Get motors from hardware map
        fLeftWheel = hwMap.dcMotor.get("FL");
        fRightWheel = hwMap.dcMotor.get("FR");
        bLeftWheel = hwMap.dcMotor.get("BL");
        bRightWheel = hwMap.dcMotor.get("BR");
        leftLift = hwMap.get(DcMotorEx.class, "LL");
        rightLift = hwMap.get(DcMotorEx.class, "RL");
        claw = hwMap.servo.get("Claw");
        wrist = hwMap.servo.get("Wrist");
        lBar = hwMap.servo.get("lBar");
        rBar = hwMap.servo.get("rBar");
        lArm = hwMap.servo.get("lArm");
        rArm = hwMap.servo.get("rArm");
        lDiff = hwMap.servo.get("lDiff");
        rDiff = hwMap.servo.get("rDiff");

        // Set Direction
        fRightWheel.setDirection(DcMotor.Direction.REVERSE);
        fLeftWheel.setDirection(DcMotor.Direction.FORWARD);
        bRightWheel.setDirection(DcMotor.Direction.REVERSE);
        bLeftWheel.setDirection(DcMotor.Direction.FORWARD);
        leftLift.setDirection(DcMotor.Direction.FORWARD);
        rightLift.setDirection(DcMotor.Direction.REVERSE);

        // Set run mode
        fRightWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fLeftWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bRightWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bLeftWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set brakes
        fRightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fLeftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bRightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bLeftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Get webcam from hardware map
        // camera = hwMap.get(WebcamName.class, "webcam");

        Stop();
    }

    public void Stop() {
        fRightWheel.setPower(0);
        fLeftWheel.setPower(0);
        bRightWheel.setPower(0);
        bLeftWheel.setPower(0);
        leftLift.setPower(0);
        rightLift.setPower(0);
        claw.setPosition(0);
        wrist.setPosition(0); //originally .825
        lArm.setPosition(0);
        rArm.setPosition(0);
        rDiff.setPosition(.5);
        lDiff.setPosition(.5);
    }
}
