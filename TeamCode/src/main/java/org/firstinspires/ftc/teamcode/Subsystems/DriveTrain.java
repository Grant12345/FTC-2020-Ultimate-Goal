package org.firstinspires.ftc.teamcode.Subsystems;

import org.firstinspires.ftc.teamcode.Util.Sensors.IMU;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

public class DriveTrain {

    HardwareMap HwMap;

    //Motor Declarations
    public DcMotor leftFrontMotor;
    public DcMotor leftBackMotor;
    public DcMotor rightFrontMotor;
    public DcMotor rightBackMotor;


    public void init(HardwareMap hwm){
        HwMap = hwm;
        leftFrontMotor = HwMap.dcMotor.get("leftFrontMotor");
        leftBackMotor = HwMap.dcMotor.get("leftBackMotor");
        rightFrontMotor = HwMap.dcMotor.get("rightFrontMotor");
        rightBackMotor = HwMap.dcMotor.get("rightBackMotor");

        leftFrontMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBackMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void Drive(String input, int encoderTicks, double power){
        if(input.equals("STRAFE_LEFT")){
            this.leftFrontMotor.setTargetPosition(encoderTicks);
            this.leftBackMotor.setTargetPosition(-encoderTicks);
            this.rightFrontMotor.setTargetPosition(-encoderTicks);
            this.rightBackMotor.setTargetPosition(encoderTicks);

            while(anyDriveMotorsBusy()) {
                this.leftFrontMotor.setPower(power);
                this.leftBackMotor.setPower(-power);
                this.rightFrontMotor.setPower(-power);
                this.rightBackMotor.setPower(power);
            }
        }
        if(input.equals("STRAFE_RIGHT")){
            this.leftFrontMotor.setTargetPosition(-encoderTicks);
            this.leftBackMotor.setTargetPosition(encoderTicks);
            this.rightFrontMotor.setTargetPosition(encoderTicks);
            this.rightBackMotor.setTargetPosition(-encoderTicks);

            while(anyDriveMotorsBusy()){
                this.leftFrontMotor.setPower(-power);
                this.leftBackMotor.setPower(power);
                this.rightFrontMotor.setPower(power);
                this.rightBackMotor.setPower(-power);
            }
        }
        if(input.equals("FORWARD")){
            this.leftFrontMotor.setTargetPosition(encoderTicks);
            this.leftBackMotor.setTargetPosition(encoderTicks);
            this.rightFrontMotor.setTargetPosition(encoderTicks);
            this.rightBackMotor.setTargetPosition(encoderTicks);

            while(anyDriveMotorsBusy()){
                this.leftFrontMotor.setPower(power);
                this.leftBackMotor.setPower(power);
                this.rightFrontMotor.setPower(power);
                this.rightBackMotor.setPower(power);
            }
        }
        if(input.equals("REVERSE")) {
            this.leftFrontMotor.setTargetPosition(-encoderTicks);
            this.leftBackMotor.setTargetPosition(-encoderTicks);
            this.rightFrontMotor.setTargetPosition(-encoderTicks);
            this.rightBackMotor.setTargetPosition(-encoderTicks);

            while (anyDriveMotorsBusy()) {
                this.leftFrontMotor.setPower(-power);
                this.leftBackMotor.setPower(-power);
                this.rightFrontMotor.setPower(-power);
                this.rightBackMotor.setPower(-power);
            }
        }
    }

//    public void Turn(String input, double power, int degrees){
//        telemetry.addData("heading", IMU.a);
//        telemetry.update();
//        if(input.equals("TURN_LEFT")){
//            while (IMU.angles) {
//                this.leftFrontMotor.setPower(-power);
//                this.leftBackMotor.setPower(-power);
//                this.rightFrontMotor.setPower(-power);
//                this.rightBackMotor.setPower(-power);
//            }
//        }
//        if(input.equals("TURN_RIGHT")){
//            while (IMU.angles) {
//                this.leftFrontMotor.setPower(-power);
//                this.leftBackMotor.setPower(-power);
//                this.rightFrontMotor.setPower(-power);
//                this.rightBackMotor.setPower(-power);
//            }
//        }
//    }
    public void setRunMode(String input) {
        if (input.equals("STOP_AND_RESET_ENCODER")) {
            this.leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            this.leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            this.rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            this.rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        if (input.equals("RUN_WITHOUT_ENCODER")) {
            this.leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            this.leftBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            this.rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            this.rightBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        if (input.equals("RUN_USING_ENCODER")) {
            this.leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            this.leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            this.rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            this.rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        if (input.equals("RUN_TO_POSITION")) {
            this.leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            this.leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            this.rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            this.rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }

    //Returns TRUE if any drive motors are busy and FALSE if not.
    public boolean anyDriveMotorsBusy() {
        if (this.leftFrontMotor.isBusy() || this.leftBackMotor.isBusy() || this.rightFrontMotor.isBusy() || this.rightBackMotor.isBusy()) {
            return (true);
        } else {
            return (false);
        }
    }

    public void DriveTelemetry(){

    }
}