package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.AnalogSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class ArmSubsystem {

    DcMotor exm, im;

    TouchSensor ls;

    public ArmSubsystem(HardwareMap hardwareMap){
        exm = hardwareMap.get(DcMotor.class,"exm");
        im = hardwareMap.get(DcMotor.class,"im");
        ls = hardwareMap.get(TouchSensor.class, "ls");

        exm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        im.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        exm.setDirection(DcMotor.Direction.FORWARD);
        im.setDirection(DcMotor.Direction.REVERSE);

        exm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        im.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        exm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        im.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void extendIntake(double power){
        exm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        exm.setPower(power);
    }

    public boolean extMotorBusy(){
        return exm.isBusy();
    }

    public void extMotorOff(){
        exm.setPower(0);
    }

    public void encoderExtend(String direction, int counts){

        switch(direction){
            case "out":
                exm.setTargetPosition(exm.getTargetPosition() + counts);
                exm.setPower(0.5);
                break;
            case "in":
                exm.setTargetPosition(exm.getTargetPosition() - counts);
                exm.setPower(0.5);
                break;
            default:
                break;
        }
        exm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        int eTarget = Math.toIntExact(Math.round(exm.getCurrentPosition() + counts));
        exm.setTargetPosition(eTarget);
        if(counts < 0){
            exm.setPower(-0.75);
        } else {
            exm.setPower(0.75);
        }
    }

    public void intake(double power){
        im.setPower(power);
    }

    public boolean blockInGrabber(){
            return ls.isPressed();
    }

    public int extensionEncoderCounts(){return exm.getCurrentPosition();}

}
