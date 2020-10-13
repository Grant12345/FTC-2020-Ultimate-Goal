package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.teamcode.Util.*;
import org.firstinspires.ftc.teamcode.Util.Sensors.*;
import org.firstinspires.ftc.teamcode.Subsystems.*;


public class Teleop extends LinearOpMode {

    public void runOpMode(){

        DriveTrain.init(hardwareMap);
        IMU.init(hardwareMap);


        waitForStart();

        while(opModeIsActive()){

            //Mecanum Drive
            Constants.drive  = gamepad1.left_stick_y * Constants.TELEOP_LIMITER;
            Constants.strafe = gamepad1.left_stick_x * Constants.TELEOP_LIMITER;
            Constants.twist  = gamepad1.right_stick_x * Constants.TELEOP_LIMITER;

            DriveTrain.leftFrontMotor.setPower(Constants.drive + Constants.strafe + Constants.twist);
            DriveTrain.leftFrontMotor.setPower(Constants.drive - Constants.strafe + Constants.twist);
            DriveTrain.leftFrontMotor.setPower(Constants.drive - Constants.strafe - Constants.twist);
            DriveTrain.leftFrontMotor.setPower(Constants.drive + Constants.strafe - Constants.twist);
        }
    }
}
