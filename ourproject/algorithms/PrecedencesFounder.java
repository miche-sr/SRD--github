package se.oru.coordination.coordination_oru.ourproject.algorithms;

import se.oru.coordination.coordination_oru.ourproject.algorithms.ConstantAccelerationForwardModel.Behavior;
import se.oru.coordination.coordination_oru.ourproject.models.CriticalSection;
import se.oru.coordination.coordination_oru.ourproject.models.RobotReport;
import se.oru.coordination.coordination_oru.ourproject.models.Vehicle;

public class PrecedencesFounder {
    private int countHead = 0;
    private int countLock = 0;
    private int countPark = 0;
    private String debug = "";
    private int V2id = -1;
    
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
            System.out.println("\u001B[35m" + "HeadToHead " + v1.getID() + " E "+ v2.getID() + "\n" + "count" + countHead + "\u001B[0m");
            prec = false;
            if(v1.getForwardModel().getRobotBehavior()==ConstantAccelerationForwardModel.Behavior.stop){
                if (v1.getPriority()<v2.getPriority()){
                    v1.setNewWholePath();
                    System.out.println("\u001B[35m" + "Heat To Head Ricalcolo Percorso di R" + v1.getID() +"\u001B[0m");
                }
                else if( v1.getPriority()== v2.getPriority() && v1.getID() < v2.getID()) {
                    v1.setNewWholePath();
                    System.out.println("\u001B[35m" + " Heat To Head Ricalcolo Percorso di R" + v1.getID() +"\u001B[0m");
                }
                else countHead = countHead + 1;
            }
        }
        
        /**SE UNO DEI DUE GIÀ NON PUÒ FERMARSI PRIMA DI SC**/
        else if (v2.getStoppingPoint() >= te2start) {
            prec = false; 
            debug =" A";
        }
        else if (v1.getStoppingPoint() >= te1start) {prec = true; debug =" B";}

        
        /**SE ENTRAMBI IN T_stop DOVRANNO ANCORA ACCEDERE,
         SI PUÒ PASSARE A ORDINAMENTI SECONDARI EURISTICI **/
        else{
            if (v1.isCsTooClose()   // caso deadlock
                && timeAtCsStart2 == -1 && timeAtCsStart1 == -1 ){
                    if(v1.getPriority() > v2.getPriority()){prec = true; debug = "C1";}
                    else if (v1.getPriority() < v2.getPriority()){prec = true; debug = "C2";}
                    
                    if(v1.getID() > v2.getID()) {prec = true; debug = " C";}
                    else { prec = false;  debug = " D";}
                    
                    System.out.println("\u001B[35m" + "R"+v1.getID()+"-R"+v2.getID()+"\t DeadLock cross1 - prec:\t"+ prec + "\t waiting "+ countLock + "\u001B[0m");
                    countLock = countLock + 1;
                    if (countLock >= 30) {
                        prec = true; 
                        if(v2.getID() == V2id) {countLock = 0; prec = false; V2id = -1;}
                        else{
                            if (countLock == 30) V2id = v2.getID();
                            System.out.println("\u001B[33m" + "R"+v1.getID()+"-R"+v2.getID()+"\t  DeadLock  cross1 - IO VADO!" + "\u001B[0m");}
                        }
                }
            //caso standard
        
            else if (timeAtCsStart2 == -1 && timeAtCsStart1 == -1 ){
                    if(v1.getPriority() > v2.getPriority()){prec = true; debug = "E1";}
                    else if (v1.getPriority() < v2.getPriority()){prec = true; debug = "E2";}
                    
                    else if(v1.getID() > v2.getID()) {prec = true;  debug =" E3";}
                    else {prec = false;  debug =" F";}
                    
                    System.out.println("\u001B[35m" + "R"+v1.getID()+"-R"+v2.getID()+"\t DeadLock cross2 - prec:\t"+ prec  + "\u001B[0m");
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

        if(v1.getForwardModel().getRobotBehavior()== Behavior.stop) countPark = countPark+1;
        else {countHead = 0;countLock = 0;countPark=0;}
        if (countHead == 35  ||  countPark == 45 || v2.getBehavior()== Behavior.reached){
            
            System.out.println("\u001B[35m" + "Provo a ricalcolare Percorso di R" + v1.getID() +"\u001B[0m");
            v1.setNewWholePath();
            countHead = 0;
            countLock = 0;
            countPark = 0;
        }

        cs.setPrecedenza(prec);
        System.out.println("R"+v1.getID() +"-R"+v2.getID()+ " debug Prec " + prec +" "+ debug );
        // System.out.println("\u001B[35m" + "my ID: "+v1.getID()+ "  sp: " + v1.getStoppingPoint() +
        // " \nother Id: " +v2.getID()+"  sp:" + v2.getStoppingPoint() + "\n"+
        // cs.getTe1Start()+ ": "+ v1.getTruncateTimes().get(cs.getTe1Start()) + "  -  "+ te2start+": "+v2.getTruncateTimes().get(cs.getTe2Start()) +
        //  "\n flag:" + v1.isCsTooClose() + " precedenza calcolata "+prec +"\n "+"\u001B[0m");

        return prec;
    }
	
	
}