// from "getCriticalSections" in "AbstractTrajectoryEnvelopeCoordinator": line 1169
package se.oru.coordination.coordination_oru.ourproject.algorithms;

import se.oru.coordination.coordination_oru.ourproject.models.CriticalSection;
import se.oru.coordination.coordination_oru.ourproject.models.Vehicle;

import java.util.ArrayList;

import org.metacsp.multi.spatioTemporal.paths.PoseSteering;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope.SpatialEnvelope;

import com.vividsolutions.jts.geom.Geometry;


public class CriticalSectionsFounder {
	
	public CriticalSection[] findCriticalSections(Vehicle v1, Vehicle v2) {

		ArrayList<CriticalSection> css = new ArrayList<CriticalSection>();
		
		SpatialEnvelope se1 = v1.getTrajectoryEnvelope().getSpatialEnvelope();
		SpatialEnvelope se2 = v2.getTrajectoryEnvelope().getSpatialEnvelope();

		double smallestRobotDimension = Math.min(se1.getFootprint().getArea(), se2.getFootprint().getArea());
		Geometry shape1 = se1.getPolygon();
		Geometry shape2 = se2.getPolygon();
		
		if (shape1.intersects(shape2)) {
			PoseSteering[] path1 = se1.getPath();
			PoseSteering[] path2 = se2.getPath();

			/*if (checkEscapePoses) {		// noi non passiamo l'intero percorso, non dovrebbe servire
				//Check that there is an "escape pose" along the paths 
				boolean safe = false;
				for (int j = 0; j < path1.length; j++) {
					//Geometry placement1 = te1.makeFootprint(path1[j]);
					Geometry placement1 = TrajectoryEnvelope.getFootprint(se1.getFootprint(), path1[j].getPose().getX(), path1[j].getPose().getY(), path1[j].getPose().getTheta());
					if (!placement1.intersects(shape2)) {
						safe = true;
						break;
					}
				}
				if (path1.length == 1 || path2.length == 1) safe = true;
				if (!safe) {
					metaCSPLogger.severe("** WARNING ** Cannot coordinate as one envelope is completely overlapped by the other!");
					metaCSPLogger.severe("** " + te1 + " <--> " + te2);
					//throw new Error("Cannot coordinate as one envelope is completely overlapped by the other!");
				}

				safe = false;
				for (int j = 0; j < path2.length; j++) {
					//Geometry placement2 = te2.makeFootprint(path2[j]);
					Geometry placement2 = TrajectoryEnvelope.getFootprint(se2.getFootprint(), path2[j].getPose().getX(), path2[j].getPose().getY(), path2[j].getPose().getTheta());
					if (!placement2.intersects(shape1)) {
						safe = true;
						break;
					}
				}
				if (path1.length == 1 || path2.length == 1) safe = true;
				if (!safe) {
					metaCSPLogger.severe("** WARNING ** Cannot coordinate as one envelope is completely overlapped by the other!");
					metaCSPLogger.severe("** " + te1 + " <--> " + te2);
					//throw new Error("Cannot coordinate as one envelope is completely overlapped by the other!");
				}
			}*/

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
				for (int j = 0; j < path1.length; j++) {
					Geometry placement1 = TrajectoryEnvelope.getFootprint(se1.getFootprint(), path1[j].getPose().getX(), path1[j].getPose().getY(), path1[j].getPose().getTheta());
					int jAbs = j+v1.getPathIndex();		// LO SI RIPORTA RISPETTO A INDICE ASSOLUTO
					if (!started && placement1.intersects(g)) {		// CALCOLO INIZIO S.C.
						started = true;
						te1Starts.add(jAbs);
					}
					else if (started && !placement1.intersects(g)) {// NON INTERSECA XK IMPRONTA È USCITA DA SC
						te1Ends.add(jAbs-1 > 0 ? jAbs-1 : 0);
						started = false;
					}
					if (started && j == path1.length-1) {
						te1Ends.add(jAbs);
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
					}
				}
				for (int k1 = 0; k1 < te1Starts.size(); k1++) {
					for (int k2 = 0; k2 < te2Starts.size(); k2++) {
						//if (te1Ends.get(k1) >= Math.max(0, minStart1) && te2Ends.get(k2) >= Math.max(0, minStart2)) {
							CriticalSection oneCS = new CriticalSection(v1, v2, te1Starts.get(k1), te2Starts.get(k2), te1Ends.get(k1), te2Ends.get(k2));
							//css.add(oneCS);
							cssOneIntersectionPiece.add(oneCS);
						//}
							
					}					
				}
				/*
				//pre-filter obsolete critical sections to avoid merging them with the new computed.
				te1Starts.clear();
				te2Starts.clear();
				te1Ends.clear();
				te2Ends.clear();
				for (CriticalSection cs : cssOneIntersectionPiece) {
					te1Starts.add(cs.getTe1Start());
					te2Starts.add(cs.getTe2Start());
					te1Ends.add(cs.getTe1End());
					te2Ends.add(cs.getTe2End());
				}
								
				// SPURIOUS INTERSECTIONS (can ignore)
				if (te1Starts.size() == 0 || te2Starts.size() == 0) {
					cssOneIntersectionPiece.clear();
				}

				// ASYMMETRIC INTERSECTIONS OF ENVELOPES
				// There are cases in which there are more starts along one envelope than along the other
				// (see the Epiroc underground mining example).
				// These "holes" may or may not be big enough to accommodate a robot. Those that are not
				// should be filtered, as they falsely indicate that the critical section ends for a little bit
				// before restarting. Because of this, such situations may lead to collision.
				// Here, we take a conservative approach: instead of verifying whether
				// the "hole" is big enough to really accommodate a robot so that it does not collide with
				// the other envelope, we simply filter out all of these cases. We do this by joining the
				// critical sections around holes.
				else if (te1Starts.size() != te2Starts.size()) {
					if (te1Starts.size() == 0 || te2Starts.size() == 0) System.out.println("CRAP: te1Starts is " + te1Starts + " and te2Starts is " + te2Starts);
					metaCSPLogger.info("Asymmetric intersections of envelopes for Robot" + te1.getRobotID() + ", Robot" + te2.getRobotID() + ":");
					metaCSPLogger.info("   Original : " + cssOneIntersectionPiece);
					CriticalSection oldCSFirst = cssOneIntersectionPiece.get(0);
					CriticalSection oldCSLast = cssOneIntersectionPiece.get(cssOneIntersectionPiece.size()-1);
					CriticalSection newCS = new CriticalSection(te1, te2, oldCSFirst.getTe1Start(), oldCSFirst.getTe2Start(), oldCSLast.getTe1End(), oldCSLast.getTe2End());				
					cssOneIntersectionPiece.clear();
					cssOneIntersectionPiece.add(newCS);
					metaCSPLogger.info("   Refined  : " + cssOneIntersectionPiece);
				}
				*/
				css.addAll(cssOneIntersectionPiece);
				
			}
		}
		
		return css.toArray(new CriticalSection[css.size()]);
	}

}
