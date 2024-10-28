package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ElevatorSubsystem{

    DcMotor elm, erm;

    public ElevatorSubsystem(HardwareMap hardwareMap){

        elm = hardwareMap.get(DcMotor.class,"elm");
        erm = hardwareMap.get(DcMotor.class,"erm");

        elm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        erm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        elm.setDirection(DcMotor.Direction.REVERSE);
        erm.setDirection(DcMotor.Direction.FORWARD);

        elm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        erm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        elm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        erm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void elevator(double power){
        if(elm.getMode() != DcMotor.RunMode.RUN_USING_ENCODER){
            elm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            erm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        elm.setPower(power);
        erm.setPower(power);
    }

    public void encoderElevator(double counts){
        if(elm.getMode() != DcMotor.RunMode.RUN_TO_POSITION){
            elm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            erm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        int lTarget = Math.toIntExact(Math.round(elm.getCurrentPosition() + counts));
        int rTarget = Math.toIntExact(Math.round(elm.getCurrentPosition() + counts));
        elm.setTargetPosition(lTarget);
        erm.setTargetPosition(rTarget);
        if(counts < 0){
            elm.setPower(-0.75);
            erm.setPower(-0.75);
        }else{
            elm.setPower(0.75);
            erm.setPower(0.75);
        }
    }

    public double leftEncoderCounts(){
        return elm.getCurrentPosition();
    }

    public double rightEncoderCounts(){
        return erm.getCurrentPosition();
    }

}
