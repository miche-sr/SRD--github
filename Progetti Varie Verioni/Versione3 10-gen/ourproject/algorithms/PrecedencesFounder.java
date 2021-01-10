package se.oru.coordination.coordination_oru.ourproject.algorithms;

import se.oru.coordination.coordination_oru.ourproject.models.CriticalSection;
import se.oru.coordination.coordination_oru.ourproject.models.RobotReport;
import se.oru.coordination.coordination_oru.ourproject.models.Vehicle;

public class PrecedencesFounder {
	
	public Boolean ComputePrecedences(CriticalSection cs) {
        Boolean prec;
        Vehicle v1 = cs.getVehicle1();
        RobotReport v2 = cs.getVehicle2();
        
        int te1start = cs.getTe1Start();
        int te1end = cs.getTe1End();
        int te2start = cs.getTe2Start();
        int te2end = cs.getTe2End();
        
        System.out.println("my ID: "+v1.getID()+"\t PathInd2: " +v2.getPathIndex()+"\n"+v2.getTruncateTimes()+"\n"+te2start+"\n"+te2end);
        
        double timeAtCsStart1 = v1.getTruncateTimes().get(cs.getTe1Start());
        double timeAtCsEnd1 = v1.getTruncateTimes().get(cs.getTe1End());
        double timeAtCsStart2 = v2.getTruncateTimes().get(cs.getTe2Start());
        double timeAtCsEnd2 = v2.getTruncateTimes().get(cs.getTe2End());
    	double braking1 = timeAtCsEnd2 - timeAtCsStart1;
    	double braking2 = timeAtCsEnd1 - timeAtCsStart2;

        if (v1.getStoppingPoint() >= te1start && v2.getStoppingPoint() >= te2start){
            System.out.println("ATTENZIONE, SBATTONO " + v1.getID() + " E "+ v2.getID() + "\n");
            prec = false;
        }
        
        /**SE UNO DEI DUE GIÀ NON PUÒ FERMARSI PRIMA DI SC**/
        else if (v2.getStoppingPoint() >= te2start) prec = false;
        else if (v1.getStoppingPoint() >= te1start) prec = true;

        /**SE ENTRAMBI IN T_stop DOVRANNO ANCORA ACCEDERE,
         SI PUÒ PASSARE A ORDINAMENTI SECONDARI EURISTICI **/
        else{
        	if (timeAtCsStart2 == -1) prec = true;
        	else if (timeAtCsStart1 == -1) prec = false;
        	else if (braking1 < 0 || braking2 < 0) prec = true;
        	else if (v1.getPriority() > v2.getPriority()) prec = true;
            else if (v1.getPriority() < v2.getPriority()) prec = false;
            else{	// A PARITÀ DI PRIORITÀ, SI PROCEDE PER DISTANZA TEMPORALE
            	if (braking1 > braking2) prec = true;
            	else if (braking1 < braking2) prec = false;
            	else { // if same brakingtime, then we use id
            		if(v1.getID() > v2.getID()) prec = true;
            		else prec = false;
            	}

            }
        }
        
        return prec;
    }
	
	
}