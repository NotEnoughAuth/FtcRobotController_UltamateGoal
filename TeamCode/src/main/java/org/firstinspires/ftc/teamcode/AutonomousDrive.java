package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "BlueAutonomous")
public class AutonomousDrive extends LinearOpMode {
    RobotDrive robot = new RobotDrive();

    public void runOpMode() {
        robot.initializeRobot(hardwareMap, telemetry, RobotDrive.allianceColor.blue);


        waitForStart();


        sleep(25000);

        robot.driveEncoder(57);
        robot.wobbleClaw.setPosition(1);
        robot.wobbleArm.setPower(1);

        while(robot.armTrigger.getState())
        {
            telemetry.addData("Button?", robot.armTrigger.getState());

        }

        robot.wobbleArm.setPower(0);
        robot.wobbleClaw.setPosition(0);

//            telemetry.addData("Red: ", robot.colorSensor.red());
//            telemetry.addData("Green: ", robot.colorSensor.green());
//            telemetry.addData("Blue: ", robot.colorSensor.blue());
            telemetry.addData("Distance: ", robot.dist.getDistance(DistanceUnit.INCH));
            telemetry.update();
    }
}
