package org.usfirst.frc.team3035.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI.Port;

public class turnPID {
	
	AHRS ahrs;
	double P=0,I=0,D=0;
	double kP,kI,kD;
	double target;
	turnPID(double kPs,double kIs,double kDs){
		kP = kPs;
		kI = kIs; 
		kD = kDs;
		ahrs = new AHRS(Port.kMXP);
	}

	public void turn(double angle) {
		target=angle;
	}
	public double get() {
		D=(((target-ahrs.getAngle()))-P);
		P=target-ahrs.getAngle();
		I+=(target-ahrs.getAngle());
		return D*kD+P*kP+I*kI;
	}
}


/*
	Make an object of this class
	Call turn angle value
	Have the get method on a while loop
	Turn manually in tele-op
*/