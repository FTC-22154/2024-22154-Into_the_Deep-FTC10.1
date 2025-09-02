package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

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
        public int pivotSpecIntakePos = -750;
        public int pivotSpecDeliverPos = 1800;
        public int pivotSmplInPos = pivotSpecIntakePos;
        public int pivotSmplDeliverPos = 1800;
        public int pivotArmUpLimit = 1850;
        public int pivotDownLimit = (pivotSmplInPos - 40);
        public int pivotR2PIncrements = 200;
        public int pivotR2PCloseEnough = 10;
        public double wristSmplIn = 0.1;
        public double wristSpecIn = 0.45;
        public double wristDeliverPos = 0.72;
        public double extendIntakePwr = 0.5;
        public double pivotRoutinesPwr = 0.8;
        public double pivotManualPwr = 0.4;
        public int extendHighPivotMax = 2400;
        public int extendLowPivotMax = 1100;
        public int extendRetractLimit = 0;
        public int extendR2PIncrements = 100;
        public int extendSpecDeliverPos = 1100;
        public int extendSmplDeliverPos = 2100;
        public int extendRetractPos = 50;
        public int extendIsSafe2PivotPos = 1500;
        public int pivotWait4ExtendTime = 500;
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

        //wristServo.setPosition(0.72);

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

    public void extendIntake(double CtrlInput, int extendCodeTgtPos){

        int extendMotorCurPos = exm.getCurrentPosition();
        if(extendMotorCurPos <= PARAMS.extendRetractLimit && CtrlInput < 0.01){
            exm.setPower(0);
        }else if(extendMotorCurPos >= PARAMS.extendHighPivotMax && CtrlInput > 0.01){
            exm.setPower(0);
        }/*else if(extendMotorCurPos > PARAMS.extendLowPivotMax && CtrlInput > 0 && pivotMotor.getTargetPosition() <= 0) {
            exm.setPower(PARAMS.extendIntakePwr);
            exm.setTargetPosition(PARAMS.extendLowPivotMax);
        } */else if(CtrlInput == 0.0111 && extendCodeTgtPos != 99999){
            exm.setPower(PARAMS.extendIntakePwr);
            exm.setTargetPosition(extendCodeTgtPos);
        } else {
            if(CtrlInput > 0.03) {
                exm.setPower(PARAMS.extendIntakePwr);
                exm.setTargetPosition(extendMotorCurPos + PARAMS.extendR2PIncrements);
            } else if(CtrlInput < -0.03){
                exm.setPower(PARAMS.extendIntakePwr);
                exm.setTargetPosition(extendMotorCurPos - PARAMS.extendR2PIncrements);
            }
        }
    }

    public boolean extendMotorBusy(){ return exm.isBusy(); }
    public boolean pivotMotorBusy(){ return pivotMotor.isBusy(); }


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
        extendIntake(0.0111,PARAMS.extendRetractPos);
        if(exm.getCurrentPosition() >= PARAMS.extendIsSafe2PivotPos){
            myWait(PARAMS.pivotWait4ExtendTime);
        }
        wristUpDown(PARAMS.wristSmplIn);
        pivotToPos(PARAMS.pivotSmplInPos);
    }
    public void specPickup(){
        extendIntake(0.01111,PARAMS.extendRetractPos);
        if(exm.getCurrentPosition() >= PARAMS.extendIsSafe2PivotPos){
            myWait(PARAMS.pivotWait4ExtendTime);
        }
        wristUpDown(PARAMS.wristSpecIn);
        pivotToPos(PARAMS.pivotSpecIntakePos);
    }
    public void specPlaceHigh(){
        wristUpDown(PARAMS.wristDeliverPos);
        pivotToPos(PARAMS.pivotSpecDeliverPos);
        extendIntake(0.0111,PARAMS.extendSpecDeliverPos);
    }
    public void smplPlaceHigh(){
        wristUpDown(PARAMS.wristDeliverPos);
        pivotToPos(PARAMS.pivotSmplDeliverPos);
        extendIntake(0.0111,PARAMS.extendSmplDeliverPos);
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
        }        int pivotMotorCurPos = pivotMotor.getCurrentPosition();

        pivotMotor.setPower(PARAMS.pivotRoutinesPwr);
    }

    public void pivotByPos(double ctrlPower){
        // First, stop moving if we're not trying to move, or if it's close enough to last new target
        int pivotMotorCurPos = pivotMotor.getCurrentPosition();
        if((ctrlPower == 0) /* ||  (Math.abs(pivotMotorCurPos - pivotMotor.getTargetPosition()) < PARAMS.pivotR2PCloseEnough)*/) {
            pivotMotor.setTargetPosition(pivotMotorCurPos);
            pivotMotor.setPower(0);
        } /* stop moving at lower limit */
            else
        if(pivotMotor.getCurrentPosition() <= PARAMS.pivotDownLimit && ctrlPower < 0){
            pivotMotor.setTargetPosition(pivotMotorCurPos);
            pivotMotor.setPower(0);
        } /* stop moving at lower limit */
            else if(pivotMotor.getCurrentPosition() >= PARAMS.pivotArmUpLimit && ctrlPower > 0){
            pivotMotor.setPower(0);
            pivotMotor.setTargetPosition(pivotMotor.getCurrentPosition());
        } /* Still here? OK, we're good to set/move to new Pos */
            else {
            if(ctrlPower > 0.03) {
                pivotMotor.setPower(PARAMS.pivotRoutinesPwr);
                pivotMotor.setTargetPosition(pivotMotor.getCurrentPosition() + PARAMS.pivotR2PIncrements);
            } else if(ctrlPower < -0.03){
                pivotMotor.setPower(PARAMS.pivotRoutinesPwr);
                pivotMotor.setTargetPosition(pivotMotor.getCurrentPosition() - PARAMS.pivotR2PIncrements);
            }
        }
    }
    public void pivotToPos(int position){
        //-365 & 7200
        //pivotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pivotMotor.setTargetPosition(position);
        if(pivotMotor.getTargetPosition() > pivotMotor.getCurrentPosition()){
            pivotMotor.setPower(PARAMS.pivotRoutinesPwr);
        } else {
            pivotMotor.setPower(PARAMS.pivotRoutinesPwr);
        }
    }

    public void encoderExtend(double counts){
        exm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        int eTarget = Math.toIntExact(Math.round(exm.getCurrentPosition() + counts));
        exm.setTargetPosition(eTarget);
        if(counts < 0){
            exm.setPower(PARAMS.extendIntakePwr);
        } else {
            exm.setPower(PARAMS.extendIntakePwr);
        }
    }

    public double armDist(){
        return ds.getDistance(DistanceUnit.INCH);
    }

    public void extendDistance(double distance){
    // New line of Code not finished -->    if(exm.)
        double targetDist = distance;
        if(targetDist - 0.5 > armDist()){
            exm.setPower(PARAMS.extendIntakePwr);
        } else if(targetDist +  0.5 < armDist()){
            exm.setPower(PARAMS.extendIntakePwr);
        } else {
            exm.setPower(0);
        }
    }

    private void myWait(int sleeptime) {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (timer.milliseconds() < sleeptime) {
        }
    }

//    public void intake(double power){
//        im.setPower(power);
//    }


    /* ================================================================================
        Data Fetching Functions below this section break...
    ===================================================================================*/

    public int extensionEncoderCounts(){return exm.getCurrentPosition();}
    public int extensionEncoderTarget(){return exm.getTargetPosition();}
    public int pivotGetTargetPos(){return pivotMotor.getTargetPosition();}
    public DcMotor.RunMode extendMotorGetMode(){return exm.getMode();}

    public double intakeGetServoPwr() { return intakeServoL.getPower() + intakeServoR.getPower();}


    public boolean blockInGrabber(){
        return ls.equals(true);
    }

}
