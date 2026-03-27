package Drivetrains;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Mecanum Drivetrain controller class
 * @author Xander Haemel - 31616 - 404 Not Found
 */
public class MecanumDrive {
    //decleration field
    MecanumConstants mechconstants;
    //motors
    DcMotorEx lf;
    DcMotorEx lr;
    DcMotorEx rf;
    DcMotorEx rr;



    //constructor
    public MecanumDrive(HardwareMap hardwareMap, MecanumConstants constants){
        this.mechconstants = constants;
        String [] names =  mechconstants.getMotorNames();
        lf = hardwareMap.get(DcMotorEx.class, names[0]);
        lr = hardwareMap.get(DcMotorEx.class, names[0]);
        rf = hardwareMap.get(DcMotorEx.class, names[0]);
        rr = hardwareMap.get(DcMotorEx.class, names[0]);
    }

    /**
     * uses the mecanum equation to set powers of the wheels based on inputs
     * @param drive the power to drive forward
     * @param strafe the power to strafe
     * @param turn the power to turn
     */
    public void setPowers(double drive, double strafe, double turn){
        double [] powers = calculatePower(drive, strafe, turn);
        lf.setPower(powers[0]);
        lf.setPower(powers[1]);
        lf.setPower(powers[2]);
        lf.setPower(powers[3]);
    }

    /**
     * caluclates the powers
     * @param drive the power to drive forward
     * @param strafe the power to strafe
     * @param turn the power to turn
     */
    private double [] calculatePower(double drive, double strafe, double turn){
        double lfPower = drive + strafe + turn;
        double lrPower = drive - strafe + turn;
        double rfPower = drive -strafe - turn;
        double rrPower = drive + strafe - turn;
        //format powers
        double [] calculatedPowers = {lfPower,lrPower,rfPower,rrPower};
        //return
        return  calculatedPowers;
    }


}
