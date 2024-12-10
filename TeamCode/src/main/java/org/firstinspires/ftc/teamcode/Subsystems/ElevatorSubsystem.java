package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ElevatorSubsystem{

    DcMotor elm, erm;
    Servo ehl, ehr;

    public ElevatorSubsystem(HardwareMap hardwareMap){

        elm = hardwareMap.get(DcMotor.class,"elm");
        erm = hardwareMap.get(DcMotor.class,"erm");
        ehl = hardwareMap.get(Servo.class, "ehl");
        ehr = hardwareMap.get(Servo.class, "ehr");

        elm.setDirection(DcMotor.Direction.REVERSE);
        erm.setDirection(DcMotor.Direction.FORWARD);
        ehl.setDirection(Servo.Direction.REVERSE);

//        elm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        erm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elm.setTargetPosition(0);
        erm.setTargetPosition(0);
        ehl.setPosition(0);
        ehr.setPosition(0);

        elm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        erm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void elevator(double power){
        if(elm.getMode() != DcMotor.RunMode.RUN_USING_ENCODER){
            elm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            erm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        elm.setPower(power);
        erm.setPower(power);
    }

    public void elevatorLift(double power){

        if(power > 0.03) {
            elm.setTargetPosition(elm.getCurrentPosition() + 200);
            erm.setTargetPosition(erm.getCurrentPosition() + 200);
        } else if(power < -0.03){
            elm.setTargetPosition(elm.getCurrentPosition() - 200);
            erm.setTargetPosition(erm.getCurrentPosition() - 200);
        }
        elm.setPower(power);
        erm.setPower(power);
    }

    public void encoderElevator(String direction, int counts){

        switch(direction){
            case "down":
                elm.setTargetPosition(elm.getCurrentPosition() + counts);
                erm.setTargetPosition(erm.getCurrentPosition() + counts);
                elm.setPower(0.75);
                erm.setPower(0.75);
                break;
            case "up":
                elm.setTargetPosition(elm.getCurrentPosition() - counts);
                erm.setTargetPosition(elm.getTargetPosition() - counts);
                elm.setPower(-0.75);
                erm.setPower(-0.75);
                break;
            default:
                break;
        }
        if(elm.getMode() != DcMotor.RunMode.RUN_TO_POSITION){
            elm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            erm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
//        int lTarget = Math.toIntExact(Math.round(elm.getCurrentPosition() + counts));
//        int rTarget = Math.toIntExact(Math.round(elm.getCurrentPosition() + counts));
//        elm.setTargetPosition(lTarget);
//        erm.setTargetPosition(rTarget);
//        if(counts < 0){
//            elm.setPower(-0.75);
//            erm.setPower(-0.75);
//        }else{
//            elm.setPower(0.75);
//            erm.setPower(0.75);
//        }
    }

    public double leftEncoderCounts(){
        return elm.getCurrentPosition();
    }

    public double rightEncoderCounts(){
        return erm.getCurrentPosition();
    }

    public boolean eleMotorsBusy(){
        return elm.isBusy() || erm.isBusy();
    }

    public void eleHook(double pos){
        ehl.setPosition(pos);
        ehr.setPosition(pos);
    }

    public void eleMotorsOff(){
        elm.setPower(0);
        erm.setPower(0);
    }

}
