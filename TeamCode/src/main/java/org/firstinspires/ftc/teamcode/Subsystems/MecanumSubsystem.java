package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MecanumSubsystem {

    DcMotor lf, lr, rf, rr;
    /*
    lf is left front drive
    lr is left rear drive
    rf is right front drive
    rr is right rear drive
    */

    public MecanumSubsystem (HardwareMap hardwareMap){

        //Set Motor Names
        lf = hardwareMap.get(DcMotor.class, "lf");
        lr = hardwareMap.get(DcMotor.class,"lr");
        rr = hardwareMap.get(DcMotor.class,"rr");
        rf = hardwareMap.get(DcMotor.class,"rf");

        //Set Motors to Brake Mode
        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Set Motor Directions
        lf.setDirection(DcMotor.Direction.REVERSE);
        rf.setDirection(DcMotor.Direction.REVERSE);
        rr.setDirection(DcMotor.Direction.REVERSE);
        lr.setDirection(DcMotor.Direction.REVERSE);

        //Reset Encoder Counts
        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Set Motors to Run With Encoders
//        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        lr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public int leftRearCounts(){
        return  lr.getCurrentPosition();
    }
    public int leftFrontCounts(){ return  lf.getCurrentPosition(); }
    public int rightFrontCounts(){
        return  rf.getCurrentPosition();
    }
    public int rightRearCounts(){
        return  rr.getCurrentPosition();
    }

    public void TeleOperatedDrive(double forward, double strafe, double turn) {

        if(lf.getMode() != DcMotor.RunMode.RUN_WITHOUT_ENCODER){
            lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            lr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        double[] speeds = {
                (forward + strafe + turn),
                (forward - strafe - turn),
                (forward - strafe + turn),
                (forward + strafe - turn)
        };

        double max = Math.abs(speeds[0]);
        for(int i = 0; i < speeds.length; i++) {
            if ( max < Math.abs(speeds[i]) ) max = Math.abs(speeds[i]);
        }

        if (max > 1) {
            for (int i = 0; i < speeds.length; i++) speeds[i] /= max;
        }

        lf.setPower(speeds[0]);
        rf.setPower(-1*speeds[1]);
        lr.setPower(speeds[2]);
        rr.setPower(-1*speeds[3]);
    }

    public void encoderDrive(String direction, int counts){
        if(lf.getMode() != DcMotor.RunMode.RUN_TO_POSITION){
            lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        switch (direction) {
            case "forward":
                lf.setTargetPosition(lf.getCurrentPosition() + counts);
                rf.setTargetPosition(rf.getCurrentPosition() + counts);
                lr.setTargetPosition(lr.getCurrentPosition() + counts);
                rr.setTargetPosition(rr.getCurrentPosition() + counts);
                lf.setPower(0.75);
                rf.setPower(0.75);
                lr.setPower(0.75);
                rr.setPower(0.75);
                break;
            case "backward":
                lf.setTargetPosition(lf.getCurrentPosition() - counts);
                rf.setTargetPosition(rf.getCurrentPosition() - counts);
                lr.setTargetPosition(lr.getCurrentPosition() - counts);
                rr.setTargetPosition(rr.getCurrentPosition() - counts);
                lf.setPower(-0.75);
                rf.setPower(-0.75);
                lr.setPower(-0.75);
                rr.setPower(-0.75);
                break;
            case "left":
                lf.setTargetPosition(lf.getCurrentPosition() + counts);
                rf.setTargetPosition(rf.getCurrentPosition() - counts);
                lr.setTargetPosition(lr.getCurrentPosition() - counts);
                rr.setTargetPosition(rr.getCurrentPosition() + counts);
                lf.setPower(0.75);
                rf.setPower(-0.75);
                lr.setPower(-0.75);
                rr.setPower(0.75);
                break;
            case "right":
                lf.setTargetPosition(lf.getCurrentPosition() - counts);
                rf.setTargetPosition(rf.getCurrentPosition() + counts);
                lr.setTargetPosition(lr.getCurrentPosition() + counts);
                rr.setTargetPosition(rr.getCurrentPosition() - counts);
                lf.setPower(-0.75);
                rf.setPower(0.75);
                lr.setPower(0.75);
                rr.setPower(-0.75);
                break;
            default:
                break;
        }
    }

    public void encoderTurn(String direction, int counts){
        if(lf.getMode() != DcMotor.RunMode.RUN_TO_POSITION){
            lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        switch (direction) {
            case "left":
                lf.setTargetPosition(lf.getCurrentPosition() - counts);
                rf.setTargetPosition(rf.getCurrentPosition() + counts);
                lr.setTargetPosition(lr.getCurrentPosition() - counts);
                rr.setTargetPosition(rr.getCurrentPosition() + counts);
                lf.setPower(-0.75);
                rf.setPower(0.75);
                lr.setPower(-0.75);
                rr.setPower(0.75);
                break;
            case "right":
                lf.setTargetPosition(lf.getCurrentPosition() + counts);
                rf.setTargetPosition(rf.getCurrentPosition() - counts);
                lr.setTargetPosition(lr.getCurrentPosition() + counts);
                rr.setTargetPosition(rr.getCurrentPosition() - counts);
                lf.setPower(0.75);
                rf.setPower(-0.75);
                lr.setPower(0.75);
                rr.setPower(-0.75);
                break;
            default:
                break;
        }
    }

}