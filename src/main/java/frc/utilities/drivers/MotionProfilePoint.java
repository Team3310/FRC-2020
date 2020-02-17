/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.utilities.drivers;

/**
 * Add your docs here.
 */
public class MotionProfilePoint {
    public double time;
	public double position;
	public double velocity;
	public double acceleration;
	
	public void initialize(double startPosition) {
		time = 0;
		position = startPosition;
		velocity = 0;
		acceleration = 0;
	}
}
