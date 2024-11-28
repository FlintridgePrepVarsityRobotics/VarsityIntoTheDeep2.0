package org.firstinspires.ftc.teamcode.Auton;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.exception.RobotCoreException;

import org.firstinspires.ftc.teamcode.Projects.HWMapBasic;

@Autonomous(name = "AutoBasic")
public class AutoBasic extends LinearOpMode{


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

        // init motor pos and encoder
        robot.rightLift.setTargetPosition(0);
        robot.leftLift.setTargetPosition(0);
        robot.rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        telemetry.addLine("" + robot.leftLift.getCurrentPosition());
        telemetry.addLine("" + robot.rightLift.getCurrentPosition());
        robot.rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.addLine("target position: " + robot.leftLift.getCurrentPosition());
        telemetry.addLine("target position: " + robot.rightLift.getCurrentPosition());
        telemetry.update();

        while(!isStarted()){

        }
        waitForStart(); //wait for play button to be pressed

//        Drive forward 1 block
//        Rise to 1
//        wrist dpad left (midpoint)
//        Drive forward from 1 to 2
//        Rise to 2
//        drive back to hook 1/5
//        release claw
//        drive back
//        lift down
//        close claw

        // Drive forward 1 tiles
        robot.fRightWheel.setPower(.46);
        robot.fLeftWheel.setPower(.46);
        robot.bRightWheel.setPower(.5);
        robot.bLeftWheel.setPower(.5);
        sleep(500);
        robot.fRightWheel.setPower(0);
        robot.fLeftWheel.setPower(0);
        robot.bRightWheel.setPower(0);
        robot.bLeftWheel.setPower(0);
        sleep(500);

        // Rise to 1
        robot.rightLift.setPower(.8);
        robot.leftLift.setPower(.8);
        robot.rightLift.setTargetPosition(-1200);
        robot.leftLift.setTargetPosition(-1200);
        sleep(3000);

        // wrist midpoint :)
        robot.wrist.setPosition(.65);
        sleep(250);

        // Forward from 1 to 2 tiles
        robot.fRightWheel.setPower(.46);
        robot.fLeftWheel.setPower(.46);
        robot.bRightWheel.setPower(.5);
        robot.bLeftWheel.setPower(.5);
        sleep(1300);
        robot.fRightWheel.setPower(0);
        robot.fLeftWheel.setPower(0);
        robot.bRightWheel.setPower(0);
        robot.bLeftWheel.setPower(0);
        sleep(500);

        // Rise to 2
        robot.rightLift.setPower(.8);
        robot.leftLift.setPower(.8);
        robot.rightLift.setTargetPosition(-2120);
        robot.leftLift.setTargetPosition(-2120);
        sleep(500);

        // wrist midpoint
        robot.wrist.setPosition(.75);
        sleep(500);

        // Drive back 1/5 of a block
        robot.fRightWheel.setPower(-.5);
        robot.fLeftWheel.setPower(-.46);
        robot.bRightWheel.setPower(-.5);
        robot.bLeftWheel.setPower(-.46);
        sleep(1050);
        robot.fRightWheel.setPower(0);
        robot.fLeftWheel.setPower(0);
        robot.bRightWheel.setPower(0);
        robot.bLeftWheel.setPower(0);
        sleep(200);
        robot.fRightWheel.setPower(-.5);
        robot.fLeftWheel.setPower(.46);
        robot.bRightWheel.setPower(-.5);
        robot.bLeftWheel.setPower(.46);
        sleep(200);
        robot.fRightWheel.setPower(.46);
        robot.fLeftWheel.setPower(.46);
        robot.bRightWheel.setPower(.5);
        robot.bLeftWheel.setPower(.5);
        sleep(2000);

        // Release claw
        robot.claw.setPosition(.415);
        sleep(500);

        /*// Drive back more
//        robot.fRightWheel.setPower(-.5);
//        robot.fLeftWheel.setPower(-.46);
//        robot.bRightWheel.setPower(-.5);
//        robot.bLeftWheel.setPower(-.46);
//        sleep(500);
//        robot.fRightWheel.setPower(0);
//        robot.fLeftWheel.setPower(0);
//        robot.bRightWheel.setPower(0);
//        robot.bLeftWheel.setPower(0);
//        sleep(200);*/

        // Close claw
        robot.claw.setPosition(0);
        sleep(200);

        // Lift down
        robot.leftLift.setTargetPosition(35);
        robot.rightLift.setTargetPosition(35);
        sleep(3000);



//        robot.rightLift.setTargetPosition(-4000);
//        robot.leftLift.setTargetPosition(-4000);
//        positions = WaitTillTargetReached(50, true);
//        robot.rightLift.setTargetPosition(rightPosition);
//        robot.leftLift.setTargetPosition(leftPosition);
//        rightPosition = positions[0];
//        leftPosition = positions[1];
//        sleep(3000);


        /*robot.fRightWheel.setPower(0);
        robot.fLeftWheel.setPower(0);
        robot.bRightWheel.setPower(0);
        robot.bLeftWheel.setPower(0);
        sleep(500);

        robot.rightLift.setTargetPosition(-4000);
        robot.leftLift.setTargetPosition(-4000);
        positions = WaitTillTargetReached(50, true);
        robot.rightLift.setTargetPosition(rightPosition);
        robot.leftLift.setTargetPosition(leftPosition);
        rightPosition = positions[0];
        leftPosition = positions[1];
        sleep(3000);
        robot.rArm.setPosition(.55);
        robot.lArm.setPosition(.45);
        sleep(350);
        robot.fRightWheel.setPower(-.5);
        robot.fLeftWheel.setPower(-.5);
        robot.bRightWheel.setPower(-.5);
        robot.bLeftWheel.setPower(-.5);
        sleep(1000);



        robot.claw.setPosition(1);
        robot.fRightWheel.setPower(0);
        robot.fLeftWheel.setPower(0);
        robot.bRightWheel.setPower(0);
        robot.bLeftWheel.setPower(0);
        sleep(500);
        robot.fRightWheel.setPower(-.5);
        robot.fLeftWheel.setPower(-.5);
        robot.bRightWheel.setPower(-.5);
        robot.bLeftWheel.setPower(-.5);
        sleep(300);
        robot.fRightWheel.setPower(0);
        robot.fLeftWheel.setPower(0);
        robot.bRightWheel.setPower(0);
        robot.bLeftWheel.setPower(0);
        sleep(300);
        robot.fRightWheel.setPower(.5);
        robot.fLeftWheel.setPower(-.5);
        robot.bRightWheel.setPower(.5);
        robot.bLeftWheel.setPower(-.5);
        sleep(400);
        robot.fRightWheel.setPower(0);
        robot.fLeftWheel.setPower(0);
        robot.bRightWheel.setPower(0);
        robot.bLeftWheel.setPower(0);
        sleep(250);
        robot.fRightWheel.setPower(-.5);
        robot.fLeftWheel.setPower(-.5);
        robot.bRightWheel.setPower(-.5);
        robot.bLeftWheel.setPower(-.5);
        sleep(2000);
        robot.fRightWheel.setPower(0);
        robot.fLeftWheel.setPower(0);
        robot.bRightWheel.setPower(0);
        robot.bLeftWheel.setPower(0);*/  //auto end


        //while(robot.right.isBusy()|| robot.left.isBusy()) {

        //telemetry.addData("Status", robot.armMotor.getCurrentPosition());
        // telemetry.update();
        //}



    }
    public void preset1 (){






    }
    int[] WaitTillTargetReached(int tolerance, boolean lock){
        int leftDifference = Math.abs(robot.leftLift.getTargetPosition() - robot.leftLift.getCurrentPosition());
        int rightDifference = Math.abs(robot.rightLift.getTargetPosition() - robot.rightLift.getCurrentPosition());
        int check=102930293;
        while(leftDifference > tolerance || rightDifference > tolerance)

        {

            leftDifference = Math.abs(robot.leftLift.getTargetPosition() - robot.leftLift.getCurrentPosition());
            rightDifference = Math.abs(robot.rightLift.getTargetPosition() - robot.rightLift.getCurrentPosition());

            robot.leftLift.setPower(0.5);
            robot.rightLift.setPower(0.5);
            if (check == robot.rightLift.getCurrentPosition() + robot.leftLift.getCurrentPosition()) {
                break;
            }
            else {
                check = robot.rightLift.getCurrentPosition() + robot.leftLift.getCurrentPosition();
            }
            sleep(1);
            int a = robot.rightLift.getCurrentPosition();
            int c = robot.leftLift.getCurrentPosition();
            telemetry.addLine("current position: " + a + "," + c);
            telemetry.addLine("target position: " + robot.leftLift.getTargetPosition());
            telemetry.update();

        }
        int a = robot.rightLift.getCurrentPosition();
        int c = robot.leftLift.getCurrentPosition();
        telemetry.addLine("current position: " + a + "," + c);
        telemetry.addLine("target position: " + robot.leftLift.getTargetPosition());
        telemetry.update();
        int[] positions = new int[] {a,c};


        if(!lock)
        {
            robot.leftLift.setPower(0);
            robot.rightLift.setPower(0);
        }
        return(positions);

    }
    private void cycle() {

        //specimen scoring

//if (gamepad2.) {
//    robot.rBar.setPosition(0);
//    robot.lBar.setPosition(1);
//    robot.rArm.setPosition(0);       //Extend fourbar, extend arm, move wrist to face downward
//    robot.lArm.setPosition(1);
//    robot.claw.setPosition(0);
//}
//
//if (gamepad2.) {
//    robot.rBar.setPosition(1);
//    robot.lBar.setPosition(0);
//    robot.rArm.setPosition(1);      //Retract fourbar, retract arm, have claw neutral(same direction as arm)
//    robot.lArm.setPosition(0);
//    robot.claw.setPosition(.5);
//}
//
//if (gamepad2.) {
//    robot.rBar.setPosition(1);
//    robot.lBar.setPosition(0);
//    robot.rArm.setPosition(1);       //retract fourbar, retract arm, have claw face slightly toward the ground
//    robot.lArm.setPosition(0);
//    robot.wrist.setPosition(1);
//}

//if (gamepad2.) {
//    robot.rBar.setPosition();
//    robot.lBar.setPosition();       //preset for pickup from wall from human player
//    robot.rArm.setPosition();       //retract fourbar, retract arm, wrist tilt slightly toward ground, lift rise to specimen height, then lift rises again after grabbing
//    robot.lArm.setPosition();
//    robot.wrist.setPosition();
//    robot.leftLift.setPower();
//    robot.rightLift.setPower();
//    robot.claw.setPosition();
//    robot.leftLift.setPower();
//    robot.rightLift.setPower();
    }
}


