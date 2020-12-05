package se.oru.coordination.coordination_oru.ourproject.models;

public class CriticalSection {
	private int te1Start = -1;
	private int te2Start = -1;
	private int te1End = -1;
	private int te2End = -1;
	//private int te1Break = -1;
	//private int te2Break = -1;
	private Vehicle v2;
	
	public CriticalSection(Vehicle v2, int te1Start, int te2Start, int te1End, int te2End) {
		this.te1Start = te1Start;
		this.te2Start = te2Start;
		this.te1End = te1End;
		this.te2End = te2End;
		this.v2 = v2;
	}
	
	public int getTe1Start() {
		return te1Start;
	}
	public void setTe1Start(int te1Start) {
		this.te1Start = te1Start;
	}
	public int getTe2Start() {
		return te2Start;
	}
	public void setTe2Start(int te2Start) {
		this.te2Start = te2Start;
	}
	public int getTe1End() {
		return te1End;
	}
	public void setTe1End(int te1End) {
		this.te1End = te1End;
	}
	public int getTe2End() {
		return te2End;
	}
	public void setTe2End(int te2End) {
		this.te2End = te2End;
	}
	public Vehicle getVehicle2() {
		return v2;
	}
	public void setVehicle2(Vehicle v2) {
		this.v2 = v2;
	}
	

}
