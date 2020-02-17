/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.utilities.drivers;

import java.text.DecimalFormat;
import java.util.Hashtable;

/**
 * Add your docs here.
 */
public class MotionProfileCache {
    private Hashtable<String, MotionProfileBoxCar> cache;
	private DecimalFormat df = new DecimalFormat("#.000"); 
	private static MotionProfileCache instance;

	private MotionProfileCache() {
		cache = new Hashtable<String, MotionProfileBoxCar>();
	}
	
	public String addMP(double startDistance, double targetDistance, double maxVelocity, double itp, double t1, double t2) {
		String key = generateKey(startDistance, targetDistance, maxVelocity, itp, t1, t2);
		
		if (!cache.containsKey(key)) {
			MotionProfileBoxCar mp = new MotionProfileBoxCar(startDistance, targetDistance, maxVelocity, itp, t1, t2);
			this.addMP(key, mp);
		}
		
		return key;
	}
	
	public void addMP(String key, MotionProfileBoxCar mp) {
		cache.put(key, mp);
	}
	
	public MotionProfileBoxCar getMP(String key) {
		return cache.get(key);
	}
	
	public static MotionProfileCache getInstance() {
		if(instance == null) {
			instance = new MotionProfileCache();
		}
		return instance;
	}
	
	public void release() {
		instance = null;
	}
	
	private String generateKey(double startDistance, double targetDistance, double maxVelocity, double itp, double t1, double t2) {
		return df.format(startDistance) + df.format(targetDistance) + df.format(maxVelocity) + df.format(itp) + df.format(t1) + df.format(t2);
	}
}
