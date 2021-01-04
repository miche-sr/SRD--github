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
        int te1end = cs.getTe1End();
        int te2start = cs.getTe2Start();
        int te2end = cs.getTe2End();
        
        double timeAtCsStart1 = v1.getTruncateTimes().get(cs.getTe1Start());
        double timeAtCsEnd1 = v1.getTruncateTimes().get(cs.getTe1End());
        double timeAtCsStart2 = v2.getTruncateTimes().get(cs.getTe2Start());
        double timeAtCsEnd2 = v2.getTruncateTimes().get(cs.getTe2End());
        double braking1 = timeAtCsEnd2 - timeAtCsStart1;
        double braking2 = timeAtCsEnd1 - timeAtCsStart2;
                
        //System.out.println("inizio CS "+te1start+" al tempo "+timeAtCsStart1);
    //        if (v1.getStoppingPoint() > te1start && v2.getStoppingPoint() > te2start){
    //            // DA EVITARE IN OGNI CASO PER DIMENSIONAMENTO!
    //        }
        
        /*SE UNO DEI DUE GIÀ NON PUÒ FERMARSI PRIMA DI SC*/
        if (v2.getStoppingPoint() > te2start) { prec = false; System.out.println("altro entrerebbe"); }
        else if (v1.getStoppingPoint() > te1start) { prec = true; System.out.println("io entrerei"); }

        /**SE ENTRAMBI IN T_stop DOVRANNO ANCORA ACCEDERE,
         SI PUÒ PASSARE A ORDINAMENTI SECONDARI EURISTICI **/
        else{
            if (v1.getPriority() > v2.getPriority()) prec = true;
            else if (v1.getPriority() < v2.getPriority()) prec = false;
            else{	// A PARITÀ DI PRIORITÀ, SI PROCEDE PER DISTANZA TEMPORALE
                System.out.println("medesima priorità");
                if (timeAtCsEnd1<timeAtCsStart2) prec = true;
                else if (timeAtCsStart2 == -1) prec = true;
                else if (braking1 < braking2) {prec = true; System.out.println("nel frenaare");}
                else prec = false;	// TODO il caso ==

            }
        }
        
        return prec;
    }
}
// public class PrecedencesFounder {
	
// 	public Boolean ComputePrecedences(CriticalSection cs) {
//         Boolean prec;
//         Vehicle v1 = cs.getVehicle1();
//         Vehicle v2 = cs.getVehicle2();
//         int pathIndx1 = v1.getPathIndex();
//         int pathIndx2 = v2.getPathIndex();
//         int te1start = cs.getTe1Start();
//         int te2start = cs.getTe2Start();
//         int cp1 = v1.getCriticalPoint();
//         int cp2 = v2.getCriticalPoint();

//         if (cp1 > te1start && cp2 > te2start){
//             // com'è possibile?
//         }
//         if (cp1 > te1start){
//             prec = true;
//         }
//         else if (pathIndx1 > cp1 && cp1 != -1){
//             prec = true;
//         }
//         else if (pathIndx2 > cp2 && cp2 != -1){
//             prec = false;
//         }
//         else if (pathIndx2 > cs.getTe2End()){
//             prec = true;
//         }
//         else if (cp2 > te2start ){
//             prec = false;
//         }
//         else{
//             if (v1.getPriority() > v2.getPriority()){
//                 prec = true;
//             }
//             else if (v2.getPriority() > v1.getPriority()){
//                 prec = false;
//             }
//             else{
//                 if ((te1start - pathIndx1) < (te2start - pathIndx2)){
//                     prec = true;
//                 }
//                 else{
//                     prec = false;
//                 }
//             }

//         }
        
//         return prec;
//     }
// }