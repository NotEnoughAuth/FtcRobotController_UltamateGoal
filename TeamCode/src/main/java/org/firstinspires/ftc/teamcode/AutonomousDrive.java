package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "BlueAutonomous")
public class AutonomousDrive extends LinearOpMode {
    RobotDrive robot = new RobotDrive();

    public void runOpMode() {
        robot.initializeRobot(hardwareMap, telemetry, RobotDrive.allianceColor.blue);


        waitForStart();

        robot.wobbleClaw.setPosition(1);
        //robot.wobbleArm.setPower(1);

        sleep(3000);

//            telemetry.addData("Red: ", robot.colorSensor.red());
//            telemetry.addData("Green: ", robot.colorSensor.green());
//            telemetry.addData("Blue: ", robot.colorSensor.blue());
            telemetry.addData("Distance: ", robot.dist.getDistance(DistanceUnit.INCH));
            telemetry.update();
    }
}
