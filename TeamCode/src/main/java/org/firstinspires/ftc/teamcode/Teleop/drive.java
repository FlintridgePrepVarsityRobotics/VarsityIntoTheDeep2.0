package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.Projects.HWMapBasic;

@TeleOp(name = "drive")
public class drive extends LinearOpMode {
    public HWMapBasic robot = new HWMapBasic();


    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        double speed = .7;

        waitForStart();
        boolean isSpinning = false;
        int liftPosition = 0;
        int jointPosition = 0;
        int noU = 1000;
        boolean gateOpen = false;
        boolean clawsOpen = false;

        while (opModeIsActive()) {

            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = -gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            robot.fLeftWheel.setPower(frontLeftPower*speed);
            robot.bLeftWheel.setPower(backLeftPower*speed);
            robot.fRightWheel.setPower(frontRightPower*speed);
            robot.bRightWheel.setPower(backRightPower*speed);

            // Teleop Code goes here

            // show wheel encoder values in telemetry
            telemetry.addData("fLeftWheel", robot.fLeftWheel.getCurrentPosition());
            telemetry.addData("fRightWheel", robot.fRightWheel.getCurrentPosition());
            telemetry.addData("bLeftWheel", robot.bLeftWheel.getCurrentPosition());
            telemetry.addData("bRightWheel", robot.bRightWheel.getCurrentPosition());





//            if(gamepad1.y){
//                liftPosition = robot.lift.getTargetPosition() + 1;
//               robot.lift.setPower(.5);
//               robot.lift.setTargetPosition(liftPosition);
//                //robot.lift.setTargetPosition(20);
//                //robot.lift.setPower(.75);
//
//
//
//            }
////            else{
////                robot.lift.setPower(0);
////            }
//            if(gamepad1.a){
//                robot.lift.setPower(.5);
//                liftPosition = robot.lift.getTargetPosition() - 1;
//                robot.lift.setTargetPosition(liftPosition);
//               // robot.lift.setPower(-.75);
//            }
//            else{
//                robot.lift.setPower(0);
//            }


//            if (gamepad1.x) {
//                robot.lift.setPower(.5);
//                robot.lift.setTargetPosition(0);
//                robot.ext.setTargetPosition(0);
//                robot.ext.setPower(.5);
//            }








        }

    }

    private void cycle() {

    }

}