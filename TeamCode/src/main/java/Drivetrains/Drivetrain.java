package Drivetrains;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.util.List;

/**
 * Abstract class implemented by all drivetrain classes
 * @author Sohum Arora
 */
public abstract class Drivetrain {

    public abstract void setPower(DcMotorEx motor,double power);
    public abstract void setPower(List<DcMotorEx> motors, double power);
}
