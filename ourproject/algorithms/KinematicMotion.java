package se.oru.coordination.coordination_oru.ourproject.algorithms;

import se.oru.coordination.coordination_oru.ourproject.models.Vehicle;

public class KinematicMotion {

	public void moveVehicle(Boolean prec, Vehicle v) {
		if ((v.getCriticalPoint() == -1) || (v.getPathIndex() < v.getCriticalPoint())){
			v.setPathIndex(v.getPathIndex() + 1);
		}
		v.setCriticalPoint(prec);
		if (v.getPathIndex() < v.getSpatialEnvelope().getPath().length-1){
			v.setPose(v.getSpatialEnvelope().getPath()[v.getPathIndex()].getPose());
		}
	}
	
}
