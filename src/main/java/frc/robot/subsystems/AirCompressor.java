/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AirCompressor extends SubsystemBase {
	// Internal
	private Compressor compressor;

	// Subsystem Instance
	private final static AirCompressor INSTANCE = new AirCompressor();

	// Constructor
	private AirCompressor() {
		compressor = new Compressor(0);
	}

	public static AirCompressor getInstance() {
		return INSTANCE;
	}

	public void turnCompressorOn(){
		compressor.setClosedLoopControl(true);
	}

	public void turnCompressorOff(){
		compressor.setClosedLoopControl(false);
  	}
}
