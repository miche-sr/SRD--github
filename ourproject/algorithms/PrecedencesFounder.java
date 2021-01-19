package se.oru.coordination.coordination_oru.ourproject.algorithms;

import se.oru.coordination.coordination_oru.ourproject.models.CriticalSection;
import se.oru.coordination.coordination_oru.ourproject.models.RobotReport;
import se.oru.coordination.coordination_oru.ourproject.models.Vehicle;

public class PrecedencesFounder {
    private int count = 0;
    private String debug = "";
    
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

        if (v1.getStoppingPoint() >= te1start && v2.getStoppingPoint() >= te2start){
            System.out.println("\u001B[35m" + "HeadToHead " + v1.getID() + " E "+ v2.getID() + "\n" + "count" + count+ "\u001B[0m");
            prec = false;
            if(v1.getForwardModel().getRobotBehavior()==ConstantAccelerationForwardModel.Behavior.stop){
                if( v1.getID() > v2.getID()) {
                    v1.setNewWholePath(v2);
                    count = 0;
                    System.out.println("\u001B[35m" + "Ricalcolo Percorso di R" + v1.getID() +"\u001B[0m");
                }
                else if (count == 15){
                    v1.setNewWholePath(v2);
                    System.out.println("\u001B[35m" + "Ricalcolo Percorso di R" + v1.getID() +"\u001B[0m");
                    count = 0;
                }
                else count = count+ 1;
            }
        }
        
        /**SE UNO DEI DUE GIÀ NON PUÒ FERMARSI PRIMA DI SC**/
        else if (v2.getStoppingPoint() >= te2start) {prec = false;  debug =" A";}
        else if (v1.getStoppingPoint() >= te1start) {prec = true; debug =" B";}

        
        /**SE ENTRAMBI IN T_stop DOVRANNO ANCORA ACCEDERE,
         SI PUÒ PASSARE A ORDINAMENTI SECONDARI EURISTICI **/
        else{
            if (v1.isCsTooClose()   // caso deadlock
                && timeAtCsStart2 == -1 && timeAtCsStart1 == -1 ){
                    System.out.println("\u001B[35m" + "R"+v1.getID()+"-R"+v2.getID()+"  DEADLOCK !!!!!!!!!" + "\u001B[0m");
                    if(v1.getID() > v2.getID()) {prec = true; debug = " C";}
                    else {
                        prec = false;  debug = " D";
                        count = count + 1;
                    }
                    if (count == 25){
                        v1.setNewWholePath(v2);
                        System.out.println("\u001B[35m" + "DEADLOCK Ricalcolo Percorso di R" + v1.getID() +"\u001B[0m");
                        count = 0;
                    }
            }
            //caso standard
        
            else if (timeAtCsStart2 == -1 && timeAtCsStart1 == -1 ){
                    if(v1.getID() > v2.getID()) {prec = true; debug =" E";}
                    else {prec = false; debug =" F";}
                    }
            else if (timeAtCsStart2 == -1 || timeAtCsEnd2 == -1) {prec = true; debug =" G";}
            else if (timeAtCsStart1 == -1 || timeAtCsEnd1 == -1) {prec = false; debug = " H";}
            else if (braking1 < 0 || braking2 < 0) {prec = true; debug =" I";}
            else if (v1.getPriority() > v2.getPriority()){ prec = true; debug =" L";}
            else if (v1.getPriority() < v2.getPriority()) {prec = false; debug =" M";}
            else{	// A PARITÀ DI PRIORITÀ, SI PROCEDE PER DISTANZA TEMPORALE
                if (braking1 > braking2) {prec = true; debug = " N";}
                else if (braking1 < braking2){ prec = false;  debug =" O";}
                else { // if same brakingtime, then we use id
                    if(v1.getID() > v2.getID()) {prec = true; debug =" P";} 
                    else {prec = false; debug =" Q";}
                }

            }
        
        }
        cs.setPrecedenza(prec);
        //System.out.println("R"+v1.getID()+" debug Prec" + debug);
        // System.out.println("\u001B[35m" + "my ID: "+v1.getID()+ "  sp" + v1.getStoppingPoint() +
        // " \n other Id: " +v2.getID()+"  sp" + v2.getStoppingPoint() + "\n"+
        // cs.getTe1Start()+ ": "+ v1.getTruncateTimes().get(cs.getTe1Start()) + "  -  "+cs.getTe2Start()+": "+v2.getTruncateTimes().get(cs.getTe2Start()) +
        //  "\n flag:" + v1.isCsTooClose() + " precedenza calcolata "+prec +"\n "+"\u001B[0m");

        return prec;
    }
	
	
}