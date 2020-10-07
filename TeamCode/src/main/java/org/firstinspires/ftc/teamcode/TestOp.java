package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class TestOp extends LinearOpMode {
    //setup an object to use for moving the robot
    RobotDrive robot = new RobotDrive();

    public void runOpMode(){
        //init button hit
        robot.initializeRobot(hardwareMap, telemetry, RobotDrive.allianceColor.blue);

        waitForStart();
        //run the code when the start button is pressed

        //run repeatedly once op mode is started
        while(opModeIsActive()){
            //TESTING
        }
    }
}
