package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name= "TeleOP")
public class TeleOPMode extends LinearOpMode {
    RobotDrive robot = new RobotDrive();

    public void runOpMode() {
        robot.initializeRobot(hardwareMap, telemetry, RobotDrive.allianceColor.blue);


        waitForStart();

        robot.dropArm.setPosition(1);

        while (opModeIsActive()) {
            //Gamepad 1  ***Drivetrain***
            double forward = gamepad1.left_stick_y * -1; //The y direction on the gamepad is reversed idk why
            double strafe = gamepad1.left_stick_x;
            //Using a cube to add exponential growth to the control of rotation
            double rotate = gamepad1.right_stick_x * robot.motorPower;



            if (gamepad1.left_bumper) robot.motorPower = 0.2;
            else if (gamepad1.right_bumper) robot.motorPower= 0.15;
            else robot.motorPower = 1;
            //Wheel control
            robot.mixDrive(forward, strafe, rotate);



            //Gamepad 2  ***Gun and intake***
            if (gamepad2.left_bumper) robot.intakeMotor.setPower(0.75); //intake wheel turn on
            else robot.intakeMotor.setPower(0);

            if (gamepad2.right_bumper) robot.intakeMotor.setPower(-0.75); //intake wheel turn on
            else robot.intakeMotor.setPower(0);

            if (gamepad2.right_trigger>0.1) robot.chainLift.setPower(gamepad2.right_trigger); //chain lift sets power based on the gamepad right trigger input
            else robot.chainLift.setPower(0);


            if (gamepad2.a) robot.flyWheel.setPower(1);
            else robot.flyWheel.setPower(0);

            if (gamepad2.y) robot.dropArm.setPosition(1.0); //TODO figure out how much needs to rotate
            else robot.dropArm.setPosition(0.0); //TODO figure out default position is on the servo to lock arm in place
            if (gamepad2.x) robot.wobbleClaw.setPosition(1.0);
            else robot.wobbleClaw.setPosition(0.0);

            if (gamepad2.dpad_down) robot.wobbleArm.setPower(1);
            else robot.wobbleArm.setPower(0);

            if (gamepad2.dpad_up) robot.wobbleArm.setPower(-1);
            else robot.wobbleArm.setPower(0);

//            telemetry.addData("Red: ", robot.colorSensor.red());
//            telemetry.addData("Green: ", robot.colorSensor.green());
//            telemetry.addData("Blue: ", robot.colorSensor.blue());
            telemetry.addData("Distance: ", robot.dist.getDistance(DistanceUnit.INCH));
            telemetry.update();
        }
        }

    }
