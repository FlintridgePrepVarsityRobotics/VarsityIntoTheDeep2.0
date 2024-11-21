package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Projects.HWMapBasic;

@TeleOp(name = "lift")
public class lift extends LinearOpMode {

    public HWMapBasic robot = new HWMapBasic();
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        double speed = .9;
        int rightPosition = 0;
        int leftPosition = 0;
        int noU = -8000;
        int[] positions;

        robot.rightLift.setTargetPosition(0);
        robot.leftLift.setTargetPosition(0);
        robot.rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        robot.rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        waitForStart();
        boolean isSpinning = false;
        while (opModeIsActive()) {
            boolean aButtonHeld = false;
            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = -gamepad1.left_stick_x*1; // Counteract imperfect strafing
            double rx = -gamepad1.right_stick_x;
            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double fLeftPower = (y + x + rx) / denominator;
            double bLeftPower = (y - x + rx) / denominator;
            double fRightPower = (y - x - rx) / denominator;
            double bRightPower = (y + x - rx) / denominator;


            double motorCurrent = robot.rightLift.getCurrent(CurrentUnit.AMPS);
            telemetry.addData("Motor Current (Amps):", motorCurrent);
            telemetry.update();

//gamepad1=
            if (gamepad2.left_trigger == 1) {

                rightPosition += 25;
                leftPosition += 25;

                robot.rightLift.setPower(.8);
                robot.leftLift.setPower(.8);
                robot.rightLift.setTargetPosition(rightPosition);
                robot.leftLift.setTargetPosition(leftPosition);
                int a = robot.rightLift.getCurrentPosition();
                int c = robot.leftLift.getCurrentPosition();
                telemetry.addLine("current position: " + a + "," + c);
                telemetry.addLine("target position: " + robot.leftLift.getTargetPosition());
                telemetry.update();
            }
            else if (gamepad2.right_trigger == 1) {

                rightPosition -= 25;
                leftPosition -= 25;

                robot.rightLift.setPower(.8);
                robot.leftLift.setPower(.8);
                robot.rightLift.setTargetPosition(rightPosition);
                robot.leftLift.setTargetPosition(leftPosition);
                int a = robot.rightLift.getCurrentPosition();
                int c = robot.leftLift.getCurrentPosition();
                telemetry.addLine("current position: " + a + "," + c);
                telemetry.addLine("target position: " + robot.leftLift.getTargetPosition());
                telemetry.update();



            }
//             Teleop Code goes here         }
            //drive climb wrist gamepad1 else is gamepad2
            if (gamepad1.y) {
                robot.claw.setPosition(0);
            }
            else if (gamepad1.x){
                robot.claw.setPosition(0.3);
            }

            if (gamepad1.right_bumper)
            {robot.wrist.setPosition(0);

            }
            else if (gamepad1.left_bumper)
            {robot.wrist.setPosition(1);

            }
            else if (gamepad1.dpad_right)
            {robot.wrist.setPosition(0.5);
            }

            else if(gamepad1.dpad_left){
                robot.wrist.setPosition(0.25);
            }
            if (gamepad2.x)
            {robot.lArm.setPosition(0);
                robot.rArm.setPosition(1);

            }
            else if (gamepad2.b)
            {robot.lArm.setPosition(1);
                robot.rArm.setPosition(0);

            }
            if(gamepad2.y){
                robot.rightLift.setTargetPosition(-5015);
                robot.leftLift.setTargetPosition(-5015);
                positions = WaitTillTargetReached(50, true);
                robot.rightLift.setTargetPosition(rightPosition);
                robot.leftLift.setTargetPosition(leftPosition);
                rightPosition = positions[0];
                leftPosition = positions[1];
            }




            if (gamepad2.dpad_right)
            {robot.lBar.setPosition(1);
                robot.rBar.setPosition(0);

            }
            if (gamepad2.dpad_left)
            {robot.lBar.setPosition(0);
                robot.rBar.setPosition(1);

            }
            if (gamepad2.dpad_up)
            {robot.lBar.setPosition(0.5);
                robot.rBar.setPosition(0.5);

            }
        }



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

    }
}
