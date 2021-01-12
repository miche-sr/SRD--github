// from "getCriticalSections" in "AbstractTrajectoryEnvelopeCoordinator": line 1169
package se.oru.coordination.coordination_oru.ourproject.algorithms;

import se.oru.coordination.coordination_oru.ourproject.models.CriticalSection;
import se.oru.coordination.coordination_oru.ourproject.models.RobotReport;
import se.oru.coordination.coordination_oru.ourproject.models.Vehicle;

import java.util.ArrayList;

import org.metacsp.multi.spatioTemporal.paths.PoseSteering;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope.SpatialEnvelope;

import com.vividsolutions.jts.geom.Geometry;
import com.vividsolutions.jts.geom.Polygon;


public class CriticalSectionsFounder {
	
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
						//if (te1Ends.get(k1) >= Math.max(0, minStart1) && te2Ends.get(k2) >= Math.max(0, minStart2)) {
							CriticalSection oneCS = new CriticalSection(v1, v2, te1Starts.get(k1), te2Starts.get(k2), te1Ends.get(k1), te2Ends.get(k2),csTruncated);
							//css.add(oneCS);
							cssOneIntersectionPiece.add(oneCS);
						//}
							
					}					
				}

				css.addAll(cssOneIntersectionPiece);
				
			}
		}
		
		return css.toArray(new CriticalSection[css.size()]);
	}

	public boolean csTooClose(Vehicle v, CriticalSection csOld, CriticalSection csNew){
		int end1 = csOld.getTe1End();
		int start2 = csNew.getTe2Start();
		double RobotDimesion = v.getWholeSpatialEnvelope().getFootprint().getArea();
		SpatialEnvelope SpaceBetweenCs;
		double SpaceBetweenCsDimesion;
		ArrayList<PoseSteering> path = new ArrayList<PoseSteering>();
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
}
