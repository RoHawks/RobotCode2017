package resource;

import java.awt.geom.Point2D;



public class NewVector {
	private double xVal;
	private double yVal;
	
	public NewVector(double xVal, double yVal) {
		this.xVal = xVal;
		this.yVal = yVal;
	}
	
	public NewVector(NewVector other) {
		this(other.getX(), other.getY());
	}
	
	public NewVector() {
		this(0, 0);
	}
	
	
	public static NewVector createPolar(double angle, double total) {
		NewVector v = new NewVector();
		v.setPolar(angle, total);
		return v;
	}
	
	public static NewVector add(NewVector v1, NewVector v2) {
		NewVector v = new NewVector();
		v.addCartesian(v1);
		v.addCartesian(v2);
		return v;
	}
	
	public double getX() {
		return this.xVal;
	}
	
	public double getY() {
		return this.yVal;
	}
	
	public double getMagnitude() {
		return Math.sqrt(Math.pow(xVal, 2) + Math.pow(yVal, 2));
	}
	
	public double getAngle() {
		return (((Math.toDegrees( Math.atan2(yVal, xVal) )) + 3600) % 360);
	}
	
	public Point2D.Double asPoint() {
		return new Point2D.Double(this.getX(), this.getY());
	}
	
	private void setX(double xVal) {
		this.xVal = xVal;
	}
	
	private void setY(double yVal) {
		this.yVal = yVal;
	}
	
	public void setCartesian(double xVal, double yVal) {
		this.setX(xVal);
		this.setY(yVal);
	}
	
	public void addCartesian(double xVal, double yVal) {
		this.setCartesian(this.xVal + xVal, this.yVal + yVal);
	}
	
	public void addCartesian(NewVector v) {
		this.addCartesian(v.getX(), v.getY());
	}
	

	public void setPolar(double angle, double total) {
		this.setCartesian(Math.cos(Math.toRadians(angle)) * total, Math.sin(Math.toRadians(angle)) * total);
	}
	
	public void setAngle(double angle) {
		this.setPolar(angle, this.getMagnitude());
	}
	
	public void setTotal(double total) {
		double mag = this.getMagnitude();
		if (mag > 0)
		{
			double rat = total / mag;
			this.xVal *= rat;
			this.yVal *= rat;
		}
		else
		{
			this.xVal = 0;
			this.yVal = 0;
		}
		
	}
	
	public void addPolar(double angle, double total) {
		this.setPolar(this.getAngle() + angle, this.getMagnitude() + total);
	}
	
	public void scaleTotal(double scaleAmount) {
		this.xVal *= scaleAmount;
		this.yVal *= scaleAmount;
	}
	
	public double dot(NewVector v) {
		return dot(this, v);
	}
	
	public static double dot(NewVector a, NewVector b) {
		return a.getX() * b.getX() + a.getY() * b.getY();
	}
	
	/**
	 * projection length of some NewVector onto this
	 * @param v NewVector to project onto this
	 * @return projection length of NewVector v onto this
	 */
	public double projectionLengthFrom(NewVector v) {
		return projectionLength(v, this);
	}
	
	/**
	 * projection length of this onto some other NewVector
	 * @param v NewVector to project this onto
	 * @return projection length of this onto NewVector v
	 */
	public double projectionLengthOnto(NewVector v) {
		return projectionLength(this, v);
	}
	
	/**
	 * projection length of NewVector a onto NewVector b
	 * @param a NewVector to project
	 * @param b NewVector projected onto
	 * @return length of the projection
	 */
	public static double projectionLength(NewVector a, NewVector b) {
		return dot(a, b) / b.getMagnitude();
	}
	
	public double angleBetween(NewVector v) {
		return angleBetween(this, v);
	}
	
	public static double angleBetween(NewVector a, NewVector b) {
		double cosTheta = dot(a, b) / (a.getMagnitude() * b.getMagnitude());
		double angle = Math.toDegrees( Math.acos(cosTheta) );
		angle = ResourceFunctions.putAngleInRange(angle);

		return Math.min( angle, 360-angle );
	}
	
	public String toString() {
		return String.format("Angle: %f, Total: %f, X: %f, Y: %f",
				this.getAngle(), this.getMagnitude(), this.getX(), this.getY());
	}
	
	public static NewVector normalized (NewVector v)
	{
		NewVector newVec = new NewVector (v);
		newVec.setTotal(1.0);
		return newVec;
	}
	
	public static NewVector scaled (NewVector v, double scale)
	{
		NewVector newVec = new NewVector (v);
		newVec.scaleTotal(scale);
		return newVec;
	}
	
	public static NewVector withLength (NewVector v, double len)
	{
		NewVector newVec = new NewVector (v);
		newVec.setTotal(len);
		return newVec;
	}
	
	public static NewVector projection (NewVector a, NewVector b)
	{
		double length = NewVector.projectionLength(a, b);
		return NewVector.withLength(b, length);
	}
	
	
}
