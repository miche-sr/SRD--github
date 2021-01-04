package se.oru.coordination.coordination_oru.ourproject.algorithms;

import se.oru.coordination.coordination_oru.ourproject.models.CriticalSection;
import se.oru.coordination.coordination_oru.ourproject.models.Vehicle;

public class PrecedencesFounder {
	
	public Boolean ComputePrecedences(CriticalSection cs) {
        Boolean prec;
        Vehicle v1 = cs.getVehicle1();
        Vehicle v2 = cs.getVehicle2();
        int pathIndx1 = v1.getPathIndex();
        int pathIndx2 = v2.getPathIndex();
        int te1start = cs.getTe1Start();
        int te2start = cs.getTe2Start();
        int cp1 = v1.getCriticalPoint();
        int cp2 = v2.getCriticalPoint();

        if (cp1 > te1start && cp2 > te2start){
            // com'è possibile?
        }
        if (cp1 > te1start){
            prec = true;
        }
        else if (pathIndx2 > cs.getTe2End()){
            prec = true;
        }
        else if (cp2 > te2start ){
            prec = false;
        }
        else{
            if (v1.getPriority() > v2.getPriority()){
                prec = true;
            }
            else if (v2.getPriority() > v1.getPriority()){
                prec = false;
            }
            else{
                if ((te1start - pathIndx1) < (te2start - pathIndx2)){
                    prec = true;
                }
                else{
                    prec = false;
                }
            }

        }
        
        return prec;
    }
}