package se.oru.coordination.coordination_oru.distributed.algorithms;

import java.util.Calendar;

import se.oru.coordination.coordination_oru.distributed.models.CriticalSection;
import se.oru.coordination.coordination_oru.distributed.models.RobotReport;
import se.oru.coordination.coordination_oru.distributed.models.Vehicle;
import se.oru.coordination.coordination_oru.distributed.models.Vehicle.Behavior;

public class PrecedencesFounder{
    
    private int countLock = 0;
    private int countPark = 0;
    private String debug = "";
    private boolean replan = true;
    
    public void setReplan( boolean replan ){
        this.replan = replan;
    }

    public Boolean ComputePrecedences(CriticalSection cs) {
        Boolean prec;
        Vehicle v1 = cs.getVehicle1();
        RobotReport v2 = cs.getVehicle2();

        
        
        int te1start = cs.getTe1Start();
        int te2start = cs.getTe2Start();
        double timeAtCsStart1 = v1.getTruncateTimes().get(cs.getTe1Start());
        double timeAtCsEnd1 = v1.getTruncateTimes().get(cs.getTe1End());
        double timeAtCsStart2 = v2.getTruncateTimes().get(cs.getTe2Start());
        double timeAtCsEnd2 = v2.getTruncateTimes().get(cs.getTe2End());
    	double braking1 = timeAtCsEnd2 - timeAtCsStart1;
        double braking2 = timeAtCsEnd1 - timeAtCsStart2;
        
        if(timeAtCsEnd1 == -1 ){ braking1 = 0; braking2 = 1;} //r1 is still with infinite waitingTime
        if(timeAtCsEnd2 == -1 ) {braking1 = 1; braking2 = 0;}//r2 is still with infinite waitingTime
        if(timeAtCsEnd1 == -1 && timeAtCsEnd2 == -1){ braking1 = 0; braking2 = 0;}//both still


        if (v1.getStoppingPoint() >= te1start && v2.getStoppingPoint() >= te2start){
            prec = false; debug =" A0 - emergency breaking";
            if(v1.getBehavior()==Behavior.stop){
                if (v1.getPriority()<v2.getPriority() && replan){v1.setNewWholePath(); debug = "\u001B[32m" + " A - priority Replan" +"\u001B[0m";}
                else if( v1.getPriority()== v2.getPriority() && v1.getID() < v2.getID() && replan) {v1.setNewWholePath();debug = "\u001B[32m" + " A - ID Replan" + "\u001B[0m";}
                
            }
        }
        else if (v2.getStoppingPoint() >= te2start) {prec = false; debug =" B - Rj can't stop"; countLock = 0;}
        else if (v1.getStoppingPoint() >= te1start) {prec = true; debug =" B - I can't stop";}

        //Heuristic
        else if (v1.getPriority() > v2.getPriority()){ prec = true; debug =" C - priority";}
        else if (v1.getPriority() < v2.getPriority() && v2.getBehavior()!= Behavior.waiting) {prec = false; debug =" C - not priority";}

        
        else if (braking1 > braking2) {prec = true; debug = " H - heuristic GO";}
        else if (braking1 < braking2){ prec = false;  debug =" H - heuristic STOP ";}
        
        else { 
            if(v1.getID() > v2.getID()) {prec = true; debug =" H - heuristic ID";} 
            else 
            {prec = false; debug =" H - heurustic NOT ID";}

            
            if (braking1== 0 && braking2 == 0) {
                debug = debug + "\u001B[35m" + "\t Lock "+"\u001B[0m";
                countLock = countLock + 1;
                if (countLock >= 10 && prec == false) {
                    int v2Pi = v1.getMainTable().get(v2.getID()).getPathIndex();
                    if ((te1start - v1.getPathIndex())< (te2start-v2Pi)){
                        prec = true; debug = debug +"\u001B[33m" + " - nearest"+"\u001B[0m";
                    }
                    else countLock = 0;
                }
            }

        }
   
        
        cs.setPrecedenza(prec);


        if(v1.getBehavior()== Behavior.stop && replan) countPark = countPark+1;
        else {countPark=0;}
        if (countPark == 45 || v2.getBehavior()== Behavior.reached){
            System.out.println("\u001B[35m" + "Trying to re-plan path of R" + v1.getID() +"\u001B[0m");
            v1.setNewWholePath();
            debug = " Forced - Replan";
            countLock = 0;countPark = 0;
        }

    
           // if((v1.getID()== 10))
           //System.out.println("R"+v1.getID() +"-R"+v2.getID()+ "\tdebug Prec " + prec +" \t"+ debug );


        return prec;
    }

	
}
