package se.oru.coordination.coordination_oru.ourproject.models;

import java.lang.reflect.Array;
import java.sql.Time;
import java.util.ArrayList;
import java.util.TreeSet;

public class VehicleThread implements Runnable {
	
	private Vehicle v;
	private double elapsedTrackingTime = 0.0;
	private int cp = -2;
	private int sp = -1;
	private TreeSet<CriticalSection> csT = new TreeSet<CriticalSection>(); 
	private ArrayList<Vehicle> vehicleCs = new ArrayList<Vehicle>();
	private Boolean prec = true;
	

	public VehicleThread(Vehicle v){
		this.v = v;
	}
	
	/************************** 
	****  MAIN ALGORITHM   ****
	***************************/

	public void run() {
		String List;

		try{
			while(v.getPathIndex() < v.getWholePath().length){
		
				///// MEMORY OF CRITICAL SECTIONS ////
				// if a cs has already been found and no one is inside the cs then I don't recalculate the cs //
				analisysedVehicles.clear();
				this.analisysedCs.clear();
				for (CriticalSection analisysedCs : v.getCs()){
					if (v.getPathIndex() < analisysedCs.getTe1Start() 
							&& analisysedCs.getVehicle2().getPathIndex() < analisysedCs.getTe2Start()) {
						this.analisysedCs.add(analisysedCs);
						analisysedVehicles.add(analisysedCs.getVehicle2());
					}
				if (csN.isCsTruncated()) System.out.print("ESISTONO QUESI CASI BRUTTI !!!!! \n !!!!!!!!!!!!");
				}
				
				// re-add the cs already analysed and find the cs of other vehicles
				v.clearCs();
				for (CriticalSection analisysedCs : this.analisysedCs)
					v.getCs().add(analisysedCs);
				
				List = "";
				boolean newCs = false;
				for (Vehicle vh : this.v.getNears()){
					if (!analisysedVehicles.contains(vh)) v.appendCs(vh);
						newCs = true;}
					else System.out.print("FUNZIONA SKIP \n");
					List = (List + vh.getID() + " " );
				}
				
				//// CALCULATE THE PRECEDENCES ////
				if ((newCs == true && v.getCs().size() != 0)  
					||(v.getCs().size() == 0 && v.getCriticalPoint() != -1) 
						|| cp == -2){
					prec = true;
					v.setCriticalPoint(-1);
					for (CriticalSection cs : this.v.getCs()){
						prec = cs.ComputePrecedences();
						if (prec == false)								//calculate precedence as long as I have precedence
							v.setCriticalPoint(cs);						//calcultate critical Point
							break;
					}
					v.setSlowingPoint(); // altrimenti non calcola ultimo punto critico
					cp = v.getCriticalPoint();
					if (cp == -1) cp = v.getWholePath().length;

					sp = v.getSlowingPoint();
					if (sp == 0) sp = 1;
				}
				else System.out.print("FUNZIONA");

				v.setTimes();

				//// CALCULATE NEW POSITION ////
				

				v.setPathIndex(elapsedTrackingTime);
				v.setPose(v.getWholePath()[v.getPathIndex()].getPose());
				v.setStoppingPoint();
				
				/// CALCULATE TRUNCATED TRAJECTORY ////
				v.setSpatialEnvelope();

				//// VISUALIZATION AND PRINT ////
				printLog(List, prec);
				
				v.getVisualization().addEnvelope(v.getWholeSpatialEnvelope().getPolygon(),v,"#f600f6");
				v.getVisualization().addEnvelope(v.getSpatialEnvelope().getPolygon(),v,"#efe007");
				

				v.getVisualization().displayPoint(v, cp-1, "#f60035"); //-1 perche array parte da zero
				v.getVisualization().displayPoint(v, sp-1, "#0008f6");
				v.getVisualization().displayPoint(v, v.getStoppingPoint(), "#f600b9");

				String infoCs = v.getForwardModel().getRobotBehavior().toString();
				v.getVisualization().displayRobotState(v.getSpatialEnvelope().getFootprint(), v,infoCs);

				/// SLEEPING TIME ////
				Thread.sleep(v.getTc());
				this.elapsedTrackingTime += v.getTc()*Vehicle.mill2sec;
			}
			System.out.println("\n R" + this.v.getID() + " : GOAL RAGGIUNTO" );
		}
		
		catch (InterruptedException e) {
			System.out.println("Thread interrotto");

		}	
	}


	/// FUNCTION FOR PRINTING INFORMATIONS ////	
	public void printLog(String List, Boolean prec) {
		String CsString = "";
		String Dist = String.format("%.2f", v.getDistanceTraveled());
		String Vel = String.format("%.2f", v.getVelocity());
		//String infoCs = this.infoCs();
		String infoCs = v.getForwardModel().getRobotBehavior().toString();
		if (v.getCs().size() != 0){
			int i = 0;
			for (CriticalSection cs : v.getCs()) {
				CsString = CsString + (i+1 +"° Sezione Critica" +
				"\t Mia: " + cs.getTe1Start()+"-"+cs.getTe1End() +
				"\t R" + cs.getVehicle2().getID() + ":" +
				"\t" + cs.getTe2Start()+"-"+cs.getTe2End()) + "\n";
				i += 1;
			}
		}
		else	CsString = ("0 Sezioni Critiche");
				
		System.out.println(
			"\n============================================================\n"+
			"\nInfo R" + this.v.getID() + " : \n" + 
			"Last Index: "+ v.getWholePath().length + "\t \t " + infoCs + "\n" + 
			"Vicini: "  + List + "\t \t Precedenza: " + prec +  "\n" + 
			"Path Index: " 	+ v.getPathIndex() + "\t \t Stopping Point: " + v.getStoppingPoint() + "\n" +
			"Distance: "+ Dist  + "\t \t Velocity: " + Vel + "\n" +
			"SLowing Point: " + v.getSlowingPoint() +"\t Critical Point:" + v.getCriticalPoint()+ "\n" + 
			CsString + "\n \n" +
			"Percorso Trasmesso \n" +v.getTruncateTimes() +   "\n"
			);
	}
	



}