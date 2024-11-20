package org.firstinspires.ftc.teamcode.Subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
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

    public class ElevatorAuto implements Action {
        private boolean initialized = false;
        public int targetPos;
        ElevatorAuto(int targetPos){
            this.targetPos = targetPos;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            double lpos = elm.getCurrentPosition();
            double rpos = erm.getCurrentPosition();
            if (!initialized && lpos > targetPos) {
                elm.setPower(-0.75);
                erm.setPower(-0.75);
                initialized = true;
            } else if (!initialized && lpos < targetPos) {
                elm.setPower(0.75);
                erm.setPower(0.75);
                initialized = true;
            }
            packet.put("liftPosLeft", lpos);
            packet.put("liftPosRight", rpos);
            if ((elm.getPower() == -0.75 && lpos < targetPos) || (elm.getPower() == 0.75 && lpos > targetPos)) {
                return true;
            } else {
                elm.setPower(0);
                erm.setPower(0);
                return false;
            }
        }
    }
    public Action elevatorAuto(int targetPos){
        return new ElevatorAuto(targetPos);
    }

    public double leftEncoderCounts(){
        return elm.getCurrentPosition();
    }

    public double rightEncoderCounts(){
        return erm.getCurrentPosition();
    }

}
