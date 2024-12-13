package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
        // Enter your constants below!! :) <--
        public double myTestParam = 0.0;
        public int pivotSpecIntakePos = -90;
        public int pivotSpecDeliverPos = 200;
        public int pivotSmplInPos = -980;
        public int pivotSmplDeliverPos = 1590;
        public int pivotUpLimit = 2200;
        public int pivotDownLimit = (pivotSmplInPos - 40);
        public int pivotR2PIncrements = 50;
        public int pivotR2PCloseEnough = 10;
        public double wristSmplIn = 0.2;
        public double wristSmplDeliver = 0.3;
        public double wristSpecIn = 0.2;
        public double wristSpecDeliver = 0.3;
    }

    public static ArmSubsystem.Params PARAMS = new ArmSubsystem.Params();

    DcMotor exm, pivotMotor;

    Servo lr, wristServo;

    CRServo intakeServoL, intakeServoR;

    AnalogInput ls;

    Rev2mDistanceSensor ds;

    public ArmSubsystem(HardwareMap hardwareMap){
        exm = hardwareMap.get(DcMotor.class,"exm");
        pivotMotor = hardwareMap.get(DcMotor.class,"rm");
        ls = hardwareMap.get(AnalogInput.class, "ls");
        ds = hardwareMap.get(Rev2mDistanceSensor.class, "ds");
        lr = hardwareMap.get(Servo.class,"leri");
        wristServo = hardwareMap.get(Servo.class,"ud");
        intakeServoL = hardwareMap.get(CRServo.class, "inServL");
        intakeServoR = hardwareMap.get(CRServo.class, "inServR");

        //pivotMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        wristServo.setPosition(0.72);

        intakeServoL.setDirection(CRServo.Direction.REVERSE);
        exm.setDirection(DcMotor.Direction.FORWARD);
        pivotMotor.setDirection(DcMotor.Direction.REVERSE);

        pivotMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pivotMotor.setTargetPosition(0);
        pivotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pivotMotor.setPower(0);

        //exm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        exm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        exm.setTargetPosition(0);
        exm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        exm.setPower(.5);
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

    public void wristUpDown(double position){
        wristServo.setPosition(position);
    }

    public void smplPickup(){
        wristUpDown(PARAMS.wristSmplIn);
        pivotToPos(PARAMS.pivotSmplInPos);
    }
    public void specPickup(){
        wristUpDown(PARAMS.wristSpecIn);
        pivotToPos(PARAMS.pivotSpecIntakePos);
    }
    public void specPlaceHigh(){
        wristUpDown(PARAMS.wristSpecDeliver);
        pivotToPos(PARAMS.pivotSpecDeliverPos);
    }
    public void smplPlaceHigh(){
        wristUpDown(PARAMS.wristSmplDeliver);
        pivotToPos(PARAMS.pivotSmplDeliverPos);
    }
    public void home(){
        wristUpDown(0);
        intake(0);
        pivotToPos(0);
        extend(0);
    }

    public double getRotatePos(){
        return pivotMotor.getCurrentPosition();
    }
    public double getTestParam() { return PARAMS.myTestParam; }

    public void rotatePower(double power){
        if(pivotMotor.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
            pivotMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        pivotMotor.setPower(power);
    }

    public void pivotByPos(double ctrlPower){
        int pivotMotorCurPos = pivotMotor.getCurrentPosition();
        // First, stop moving if we're not trying to move, or if it's close enough to last new target
        if((ctrlPower == 0) /* ||  (Math.abs(pivotMotorCurPos - pivotMotor.getTargetPosition()) < PARAMS.pivotR2PCloseEnough)*/) {
            pivotMotor.setTargetPosition(pivotMotorCurPos);
            pivotMotor.setPower(0);
        } /* stop moving at lower limit */
            else
        if(pivotMotor.getCurrentPosition() <= PARAMS.pivotDownLimit && ctrlPower < 0){
            pivotMotor.setTargetPosition(pivotMotorCurPos);
            pivotMotor.setPower(0);
        } /* stop moving at lower limit */
            else if(pivotMotor.getCurrentPosition() >= PARAMS.pivotUpLimit && ctrlPower > 0){
            pivotMotor.setPower(0);
            pivotMotor.setTargetPosition(pivotMotor.getCurrentPosition());
        } /* Still here? OK, we're good to set/move to new Pos */
            else {
            if(ctrlPower > 0.03) {
                pivotMotor.setPower(1);
                pivotMotor.setTargetPosition(pivotMotor.getCurrentPosition() + PARAMS.pivotR2PIncrements);
            } else if(ctrlPower < -0.03){
                pivotMotor.setPower(1);
                pivotMotor.setTargetPosition(pivotMotor.getCurrentPosition() - PARAMS.pivotR2PIncrements);
            }
        }
    }
    public void pivotToPos(int position){
        //-365 & 7200
        //pivotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pivotMotor.setTargetPosition(position);
        if(pivotMotor.getTargetPosition() > pivotMotor.getCurrentPosition()){
            pivotMotor.setPower(0.5);
        } else {
            pivotMotor.setPower(-0.5);
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
    public int pivotGetTargetPos(){return pivotMotor.getTargetPosition();}

}
