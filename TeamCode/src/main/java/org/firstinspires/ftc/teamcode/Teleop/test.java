package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.Projects.HWMapBasic;

@TeleOp(name = "test")
public class test extends LinearOpMode {
    public HWMapBasic robot = new HWMapBasic();


    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        robot.fRightWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.fLeftWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.bRightWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.bLeftWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        waitForStart();
        boolean isSpinning = false;
        int liftPosition = 0;
        int jointPosition = 0;
        int noU = 1000;
        boolean gateOpen = false;
        boolean clawsOpen = false;
        int counter = 0;
        double speed = .1;
        double armPos = .5;
        double wristPos = .5;
        double clawPos = .5;

        while (opModeIsActive()) {
            if(gamepad1.right_bumper){
                if(counter<8){
                    counter++;
                }
                else{
                    counter = 0;
                }
            }
            else if(gamepad1.left_bumper){
                if(counter>0){
                    counter--;
                }
                else{
                    counter = 8;
                }
            }
            if(counter<=5){
                if(gamepad1.a){
                    speed = .1;
                    telemetry.addData("speed", speed);
                    telemetry.update();
                }
                else if(gamepad1.x){
                    speed = .25;
                    telemetry.addData("speed", speed);
                    telemetry.update();
                }
                else if(gamepad1.y){
                    speed = .5;
                    telemetry.addData("speed", speed);
                    telemetry.update();
                }
                else if(gamepad1.b){
                    speed = 1;
                    telemetry.addData("speed", speed);
                    telemetry.update();
                }
            }
            if (counter == 0){
                telemetry.addLine("front right");
                telemetry.update();
                if(gamepad1.dpad_up){
                    robot.fRightWheel.setPower(speed);
                }
                else if(gamepad1.dpad_down){
                    robot.fRightWheel.setPower(-speed);
                }
            }
            if (counter == 1){
                telemetry.addLine("front left");
                telemetry.update();
                if(gamepad1.dpad_up){
                    robot.fLeftWheel.setPower(speed);
                }
                else if(gamepad1.dpad_down){
                    robot.fLeftWheel.setPower(-speed);
                }
            }
            if (counter == 2){
                telemetry.addLine("back right");
                telemetry.update();
                if(gamepad1.dpad_up){
                    robot.bRightWheel.setPower(speed);
                }
                else if(gamepad1.dpad_down){
                    robot.bRightWheel.setPower(-speed);
                }
            }
            if (counter == 3){
                telemetry.addLine("back left");
                telemetry.update();
                if(gamepad1.dpad_up){
                    robot.bLeftWheel.setPower(speed);
                }
                else if(gamepad1.dpad_down){
                    robot.bLeftWheel.setPower(-speed);
                }
            }
            if (counter == 4){
                telemetry.addLine("left lift");
                telemetry.update();
                if(gamepad1.dpad_up){
                    robot.leftLift.setPower(speed);
                }
                else if(gamepad1.dpad_down){
                    robot.leftLift.setPower(-speed);
                }
            }
            if (counter == 5){
                telemetry.addLine("right lift");
                telemetry.update();
                if(gamepad1.dpad_up){
                    robot.rightLift.setPower(speed);
                }
                else if(gamepad1.dpad_down){
                    robot.rightLift.setPower(-speed);
                }
            }
            if (counter == 6){
                telemetry.addLine("arm");
                telemetry.update();
                if(armPos>0){
                    if(gamepad1.y){
                        armPos-=.05;
                        sleep(750);
                    }
                }
                if(armPos<1){
                    if(gamepad1.a){
                        armPos+=.05;
                        sleep(750);
                    }
                }
                telemetry.addData("Arm position", armPos);
                telemetry.update();
                if(gamepad1.dpad_up){
                    robot.lArm.setPosition(armPos);
                    robot.rArm.setPosition(1-armPos);
                }

            }
            if (counter == 7){
                telemetry.addLine("wrist");
                telemetry.update();
                if(wristPos>0){
                    if(gamepad1.y){
                        wristPos-=.05;
                        sleep(750);
                    }
                }
                if(wristPos<1){
                    if(gamepad1.a){
                        wristPos+=.05;
                        sleep(750);
                    }
                }
                telemetry.addData("wrist position", wristPos);
                telemetry.update();
                if(gamepad1.dpad_up){
                    robot.wrist.setPosition(wristPos);

                }

            }
            if (counter == 8){
                telemetry.addLine("claw");
                telemetry.update();
                if(clawPos>0){
                    if(gamepad1.y){
                        clawPos-=.05;
                        sleep(750);
                    }
                }
                if(wristPos<1){
                    if(gamepad1.a){
                        clawPos+=.05;
                        sleep(750);
                    }
                }
                telemetry.addData("claw position", clawPos);
                telemetry.update();
                if(gamepad1.dpad_up){
                    robot.lArm.setPosition(clawPos);

                }

            }


        }

    }

    private void cycle() {

    }

}