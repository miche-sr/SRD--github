// from "getCriticalSections" in "AbstractTrajectoryEnvelopeCoordinator": line 1169
package se.oru.coordination.coordination_oru.distributed.algorithms;

import se.oru.coordination.coordination_oru.motionplanning.AbstractMotionPlanner;
import se.oru.coordination.coordination_oru.motionplanning.ompl.ReedsSheppCarPlanner;

import se.oru.coordination.coordination_oru.distributed.models.CriticalSection;
import se.oru.coordination.coordination_oru.distributed.models.RobotReport;
import se.oru.coordination.coordination_oru.distributed.models.TrafficLights;
import se.oru.coordination.coordination_oru.distributed.models.Vehicle;
import se.oru.coordination.coordination_oru.distributed.models.Vehicle.Behavior;

import java.util.ArrayList;
import java.util.HashMap;

import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope.SpatialEnvelope;
import com.vividsolutions.jts.geom.Polygon;
import com.vividsolutions.jts.geom.Geometry;
import com.vividsolutions.jts.geom.Polygon;



public class CriticalSectionsFounder {
	
	/**************
	 * REPLANNING *
	 **************/
	public Geometry makeObstacle(RobotReport r) {
		Polygon fp = r.getSpatialEnvelope().getFootprint();
		Pose p = r.getSpatialEnvelope().getPath()[0].getPose();
		Geometry obstacle = TrajectoryEnvelope.getFootprint(fp, p.getX(), p.getY(), p.getTheta());
		return obstacle;
	}
	
	public PoseSteering[] doReplanning(AbstractMotionPlanner mp, Pose fromPose, Pose[] toPose, Geometry... obstaclesToConsider) {
		if (mp == null) return null;
		mp.setStart(fromPose);
		mp.setGoals(toPose);
		if (obstaclesToConsider != null && obstaclesToConsider.length > 0) mp.addObstacles(obstaclesToConsider);
		boolean replanningSuccessful = mp.plan();
		if (!replanningSuccessful) mp.writeDebugImage();
		if (obstaclesToConsider != null && obstaclesToConsider.length > 0) mp.clearObstacles();
		if (replanningSuccessful) return mp.getPath();
		return null;
	}
	
	public PoseSteering[] rePlanPath(Vehicle v, HashMap<Integer,RobotReport> mainTable, ArrayList<Integer> nears) {
		int currentWaitingIndex = v.getPathIndex();
		Pose currentWaitingPose = v.getPose();
		Pose[] currentWaitingGoal = v.getGoal();
		PoseSteering[] oldPath = v.getWholePath();
		ArrayList<Geometry> obstacles =  new ArrayList<Geometry>();;
		AbstractMotionPlanner mp = v.getMotionPlanner();
		int count = 0;
		String List = " ";
		//Geometry ob2 = null;
		for (Integer vh : nears){
			if (mainTable.get(vh).getBehavior() == Behavior.stop
					|| mainTable.get(vh).getBehavior() == Behavior.reached
				||mainTable.get(vh).getBehavior() == Behavior.waiting){
				obstacles.add(makeObstacle(mainTable.get(vh)));
				//ob2 = makeObstacle(mainTable.get(vh));
				List = (List + vh + " " );
				count = count + 1;
			}
		}
		if(count != 0){
			Geometry[] obstaclesG = obstacles.toArray(new Geometry[obstacles.size()]);		
			System.out.println("\u001B[35m" + "Attempting to re-plan path of Robot" + v.getID() + " (with robot" + List + " as obstacle), "
					+ "with starting point in "+currentWaitingPose+"..." + "\u001B[0m");
					System.out.println("\u001B[35m" + obstaclesG + "\u001B[0m");
			
			PoseSteering[] newPath = doReplanning(mp, currentWaitingPose, currentWaitingGoal, obstaclesG);
			// System.out.println(newPath.length);
			// PoseSteering[] newCompletePath = new PoseSteering[newPath.length+currentWaitingIndex];
			if (newPath != null && newPath.length > 0) {
				PoseSteering[] newCompletePath = new PoseSteering[newPath.length+currentWaitingIndex];
				for (int i = 0; i < newCompletePath.length; i++) {
					if (i < currentWaitingIndex) newCompletePath[i] = oldPath[i];
					else newCompletePath[i] = newPath[i-currentWaitingIndex];
				}
				return newCompletePath;
			}
			else {
				System.out.println("\u001B[31m" + "Failed to re-plan path of Robot" + v.getID() + "\u001B[0m");
				return oldPath;
			}
		}
		else{
			System.out.println("\u001B[31m" + "no need to recalculate, wait a little longer R" + v.getID() + "\u001B[0m");
			return oldPath;
		}
		
	}
	
	/************************
	 * FIND CRITICAL SECTION *
	 ************************/
	public CriticalSection[] findCriticalSections(Vehicle v1, RobotReport v2) {

		ArrayList<CriticalSection> css = new ArrayList<CriticalSection>();
		
		SpatialEnvelope se1 = v1.getSpatialEnvelope();
		SpatialEnvelope se2 = v2.getSpatialEnvelope();

		double smallestRobotDimension = Math.min(se1.getFootprint().getArea(), se2.getFootprint().getArea());
		Geometry shape1 = se1.getPolygon();
		Geometry shape2 = se2.getPolygon();
		
		if (shape1.intersects(shape2)) {
			PoseSteering[] path1 = se1.getPath();
			PoseSteering[] path2 = se2.getPath();

			Geometry gc = shape1.intersection(shape2);
			ArrayList<Geometry> allIntersections = new ArrayList<Geometry>();
			if (gc.getNumGeometries() == 1) {
				allIntersections.add(gc);
			}
			else {
				for (int i = 1; i < gc.getNumGeometries(); i++) {
					Geometry prev = gc.getGeometryN(i-1);
					Geometry next = gc.getGeometryN(i);					
					if (prev.distance(next) < smallestRobotDimension) {
						allIntersections.add(prev.union(next).convexHull());
					}
					else {
						allIntersections.add(prev);
						if (i == gc.getNumGeometries()-1) allIntersections.add(next);
					}
				}
			}

			
			for (int i = 0; i < allIntersections.size(); i++) {
				ArrayList<CriticalSection> cssOneIntersectionPiece = new ArrayList<CriticalSection>();
				ArrayList<Integer> te1Starts = new ArrayList<Integer>();
				ArrayList<Integer> te1Ends = new ArrayList<Integer>();
				ArrayList<Integer> te2Starts = new ArrayList<Integer>();
				ArrayList<Integer> te2Ends = new ArrayList<Integer>();

				Geometry g = allIntersections.get(i);
				boolean started = false;
				boolean csTruncated = false;
				for (int j = 0; j < path1.length; j++) {
					Geometry placement1 = TrajectoryEnvelope.getFootprint(se1.getFootprint(), path1[j].getPose().getX(), path1[j].getPose().getY(), path1[j].getPose().getTheta());
					int jAbs = j+v1.getPathIndex();		// RE-WRITTEN W.R.T. CURRENT PATH INDEX
					if (!started && placement1.intersects(g)) {		// COMPUTE CS BEGINNING
						started = true;
						te1Starts.add(jAbs);
					}
					else if (started && !placement1.intersects(g)) {// WHEN THE FOOTPRINT EXITS CS
						te1Ends.add(jAbs-1 > 0 ? jAbs-1 : 0);
						started = false;
					}
					if (started && j == path1.length-1) {
						te1Ends.add(jAbs);
						csTruncated = true;
					}
				}
				started = false;
				for (int j = 0; j < path2.length; j++) {
					Geometry placement2 = TrajectoryEnvelope.getFootprint(se2.getFootprint(), path2[j].getPose().getX(), path2[j].getPose().getY(), path2[j].getPose().getTheta());
					int jAbs = j+v2.getPathIndex();
					if (!started && placement2.intersects(g)) {
						started = true;
						te2Starts.add(jAbs);
					}
					else if (started && !placement2.intersects(g)) {
						te2Ends.add(jAbs-1 > 0 ? jAbs-1 : 0);
						started = false;
					}
					if (started && j == path2.length-1) {
						te2Ends.add(jAbs);
						csTruncated=true;
					}
				}
				for (int k1 = 0; k1 < te1Starts.size(); k1++) {
					for (int k2 = 0; k2 < te2Starts.size(); k2++) {
							CriticalSection oneCS = new CriticalSection(v1, v2, te1Starts.get(k1), te2Starts.get(k2), te1Ends.get(k1), te2Ends.get(k2),csTruncated);
							cssOneIntersectionPiece.add(oneCS);							
					}					
				}
				css.addAll(cssOneIntersectionPiece);
			}
		}
		return css.toArray(new CriticalSection[css.size()]);
	}

	public boolean csTooClose(Vehicle v, CriticalSection csOld, CriticalSection csNew){
		int start1 = csOld.getTe1Start();
		int end1 = csOld.getTe1End();
		int start2 = csNew.getTe1Start();
		double RobotDimesion = v.getWholeSpatialEnvelope().getFootprint().getArea();
		SpatialEnvelope SpaceBetweenCs;
		double SpaceBetweenCsDimesion;
		ArrayList<PoseSteering> path = new ArrayList<PoseSteering>();
		if (start1 == v.getPathIndex()) return false;
		if (end1 < start2){
			for (int i=end1; i < start2; i++){
				path.add(v.getWholePath()[i]);
			}
			PoseSteering[] pathArray = path.toArray(new PoseSteering[path.size()]);
			SpaceBetweenCs = TrajectoryEnvelope.createSpatialEnvelope(pathArray, v.getFootprint());
			SpaceBetweenCsDimesion = SpaceBetweenCs.getPolygon().getArea();
			if(SpaceBetweenCsDimesion < RobotDimesion) 
				return true;
			else 
				return false;
		}
		else
			return true;
		
	
	
	}

	public int SmStopIndex(Vehicle v, TrafficLights Sm){
		SpatialEnvelope se1 = v.getWholeSpatialEnvelope();
		PoseSteering[] path1 = se1.getPath();
		Geometry shape1 = se1.getPolygon();
		Geometry shape2 = Sm.getCorridorPath().getPolygon();
		Geometry g = shape1.intersection(shape2);
		boolean started = false;
		int te1Start = -1;
		for (int j = v.getPathIndex(); j < path1.length; j++) {
			Geometry placement1 = TrajectoryEnvelope.getFootprint(se1.getFootprint(), path1[j].getPose().getX(), path1[j].getPose().getY(), path1[j].getPose().getTheta());
			int jAbs = j ;//+v.getPathIndex();		
			if (!started && placement1.intersects(g)) {	
				started = true;
				te1Start = jAbs;
				break;
			}
		}
		return te1Start;

	}

	public ArrayList<Integer> findCriticalSectionsAll(Vehicle v1,  ArrayList<Vehicle> vehicleList) {
		ArrayList<Integer> totalTe1Ends = new ArrayList<Integer>();
		ArrayList<Integer> totalTe1start = new ArrayList<Integer>();
		for (Vehicle v2 : vehicleList){
			if (v1.getID() != v2.getID()){

				
				SpatialEnvelope se1 = v1.getWholeSpatialEnvelope();
				SpatialEnvelope se2 = v2.getWholeSpatialEnvelope();

				double smallestRobotDimension = Math.min(se1.getFootprint().getArea(), se2.getFootprint().getArea());
				Geometry shape1 = se1.getPolygon();
				Geometry shape2 = se2.getPolygon();

				if (shape1.intersects(shape2)) {
					PoseSteering[] path1 = se1.getPath();

					Geometry gc = shape1.intersection(shape2);
					ArrayList<Geometry> allIntersections = new ArrayList<Geometry>();
					if (gc.getNumGeometries() == 1) {
						allIntersections.add(gc);
					}
					else {
						for (int i = 1; i < gc.getNumGeometries(); i++) {
							Geometry prev = gc.getGeometryN(i-1);
							Geometry next = gc.getGeometryN(i);					
							if (prev.distance(next) < smallestRobotDimension) {
								allIntersections.add(prev.union(next).convexHull());
							}
							else {
								allIntersections.add(prev);
								if (i == gc.getNumGeometries()-1) allIntersections.add(next);
							}
						}
					}

					
					for (int i = 0; i < allIntersections.size(); i++) {
						ArrayList<Integer> te1Starts = new ArrayList<Integer>();
						ArrayList<Integer> te1Ends = new ArrayList<Integer>();

						Geometry g = allIntersections.get(i);
						boolean started = false;
						for (int j = 0; j < path1.length; j++) {
							Geometry placement1 = TrajectoryEnvelope.getFootprint(se1.getFootprint(), path1[j].getPose().getX(), path1[j].getPose().getY(), path1[j].getPose().getTheta());
							int jAbs = j;//+v1.getPathIndex();		
							if (!started && placement1.intersects(g)) {		
								started = true;
								te1Starts.add(jAbs);
								totalTe1start.add(jAbs);
							}
							else if (started && !placement1.intersects(g)) {
								te1Ends.add(jAbs-1 > 0 ? jAbs-1 : 0);
								totalTe1Ends.add(jAbs-1 > 0 ? jAbs-1 : 0);
								started = false;
							}
							if (started && j == path1.length-1) {
								te1Ends.add(jAbs);
								totalTe1Ends.add(jAbs);
								
							}
						}
						
						
					}
				}
			}
			
		}
		return totalTe1start;
	}
}
