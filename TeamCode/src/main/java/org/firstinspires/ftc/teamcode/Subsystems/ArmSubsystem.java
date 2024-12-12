package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
public class ArmSubsystem {

    /*
    This sub-class exists to hold all CONSTANTS needed for ArmSubsystem functions AND makes them all
    available to configure, on-the-fly, from the robot's built-in Dashboard web page!
    http://192.168.43.1:8080/dash
    A test/example constant has already been created and gets displayed in TeleData to prove itself.

    !*! All hard-coded values, which have any possibility of needing to be tuned (such as the
    exact encoder positions of the limits of the arm extension motor,) should be replaced with
    constants in this class
     */
    public static class Params {
        public double myTestParam = 0.0;
        //your constants here!! :) <--
    }

    public static ArmSubsystem.Params PARAMS = new ArmSubsystem.Params();

    DcMotor exm, rm;

    Servo lr, ud;

    CRServo intakeServoL, intakeServoR;

    AnalogInput ls;

    Rev2mDistanceSensor ds;

    public ArmSubsystem(HardwareMap hardwareMap){
        exm = hardwareMap.get(DcMotor.class,"exm");
        rm = hardwareMap.get(DcMotor.class,"rm");
        ls = hardwareMap.get(AnalogInput.class, "ls");
        ds = hardwareMap.get(Rev2mDistanceSensor.class, "ds");
        lr = hardwareMap.get(Servo.class,"leri");
        ud = hardwareMap.get(Servo.class,"ud");
        intakeServoL = hardwareMap.get(CRServo.class, "inServL");
        intakeServoR = hardwareMap.get(CRServo.class, "inServR");

        exm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        ud.setPosition(0.72);

        exm.setDirection(DcMotor.Direction.FORWARD);
        rm.setDirection(DcMotor.Direction.REVERSE);
        intakeServoL.setDirection(CRServo.Direction.REVERSE);

        exm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        exm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        exm.setTargetPosition(0);
        exm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        exm.setPower(.5);


        rm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void intake(double power) {

        intakeServoL.setPower(power);
        intakeServoR.setPower(power);
    }

    public void extend(int position){
        exm.setTargetPosition(position);
    }

    public void extendIntake(double power){
        if(exm.getCurrentPosition() <= 100 && power < 0){
            exm.setPower(0);
        }else if(exm.getCurrentPosition() >= 2800 && power > 0){
            exm.setPower(1);
            exm.setTargetPosition(exm.getCurrentPosition());
        }else {
            if(power > 0.03) {
                exm.setPower(1);
                exm.setTargetPosition(exm.getCurrentPosition() + 200);
            } else if(power < -0.03){
                exm.setPower(1);
                exm.setTargetPosition(exm.getCurrentPosition() - 200);
            }
        }
    }

    public void extendPower(double power){
        if(exm.getMode() != DcMotor.RunMode.RUN_USING_ENCODER){
            exm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        exm.setPower(power);
    }

    public void leftRight(double position){
        lr.setPosition(position);
    }

    public void upDown(double position){
        ud.setPosition(position);
    }

    public void pickup(){
        upDown(0.3);
        rotate(-500);
    }

    public void specimenPick(){
        upDown(0.2);
        rotate(1000);
    }

    public void specimenPlaceHigh(){
        upDown(0.2);
        rotate(2000);
    }

    public void highBucket(){
        upDown(0.2);
        rotate(858);
    }

    public void home(){
        upDown(0);
        intake(0);
        rotate(0);
        extend(0);
    }
    public double rotatePos(){
        return rm.getCurrentPosition();
    }
    public double getTestParam() { return PARAMS.myTestParam; }

    public void rotatePower(double power){
        if(rm.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
            rm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        rm.setPower(power);
    }
    public void rotate(int position){
        //-365 & 7200
        rm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rm.setTargetPosition(position);
        if(rm.getTargetPosition() > rm.getCurrentPosition()){
            rm.setPower(0.5);
        } else {
            rm.setPower(-0.5);
        }
    }

    public void encoderExtend(double counts){
        exm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        int eTarget = Math.toIntExact(Math.round(exm.getCurrentPosition() + counts));
        exm.setTargetPosition(eTarget);
        if(counts < 0){
            exm.setPower(-0.75);
        } else {
            exm.setPower(0.75);
        }
    }

    public double armDist(){
        return ds.getDistance(DistanceUnit.INCH);
    }

    public void extendDistance(double distance){
    // New line of Code not finished -->    if(exm.)
        double targetDist = distance;
        if(targetDist - 0.5 > armDist()){
            exm.setPower(0.5);
        } else if(targetDist +  0.5 < armDist()){
            exm.setPower(-0.5);
        } else {
            exm.setPower(0);
        }
    }

//    public void intake(double power){
//        im.setPower(power);
//    }

    public boolean blockInGrabber(){
        return ls.equals(true);
    }

    public int extensionEncoderCounts(){return exm.getCurrentPosition();}

}
