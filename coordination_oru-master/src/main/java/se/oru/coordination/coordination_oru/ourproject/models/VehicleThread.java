package se.oru.coordination.coordination_oru.ourproject.models;
import se.oru.coordination.coordination_oru.ourproject.algorithms.*;
import se.oru.coordination.coordination_oru.ourproject.models.CriticalSection;
import java.util.*;

public class VehicleThread implements Runnable {

	
	private Vehicle v;
	
	
	public VehicleThread( Vehicle v){
		
		this.v = v;
		}
		

	
	
	public void run() {  //algorithm 1
		int i = 0;
		String List;
		Intersection intersect = new Intersection();
		ArrayList<CriticalSection> csVh = new ArrayList<CriticalSection>();
		//System.out.println("\n" + this.v.getSpatialEnvelope().getPolygon().getEnvelope());
		
			
		try{
			while(i<1){		// this will be while true
				//setPathRadius(path,id,time)
				System.out.println("\n" + "R"  + this.v.getID() + " giro "+ i + " posizione : " + this.v.getPose().getX());
				//this.v.setPose(this.v.getPose().getX() + this.v.getVelMax() ,this.v.getPose().getY() , 0);
				csVh.clear();
				List = " ";
				for (Vehicle vh : this.v.getNears()){
					csVh.addAll(intersect.getCriticalSections(v, vh));
					
					List = (List + vh.getID()+" " );
					}
				v.setCs(csVh);
				System.out.println("\n" + "List R" + this.v.getID() + " : " + List);
				System.out.println("\n" +  this.v.getID() + "  cs : " + v.getCs().get(1).getTe1End()+ "  cs2 : " + v.getCs().get(0).getTe2End());
				//System.out.println("\n R" + this.v.getID() + " " + this.v.getPose().getX());
				
				i++;
				Thread.sleep(v.getTc());
				
			}
		}
		catch (InterruptedException e) {
			System.out.println("Thread interrotto");
		}
		
	}

	
}

