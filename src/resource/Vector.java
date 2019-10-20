package resource;

import java.awt.geom.Point2D;



public class Vector {
	private double xVal;
	private double yVal;
	
	public Vector(double xVal, double yVal) {
		this.xVal = xVal;
		this.yVal = yVal;
	}
	
	public Vector(Vector other) {
		this(other.getX(), other.getY());
	}
	
	public Vector() {
		this(0, 0);
	}
	
	
	public static Vector createPolar(double angle, double total) {
		Vector v = new Vector();
		v.setPolar(angle, total);
		return v;
	}
	
	public static Vector add(Vector v1, Vector v2) {
		Vector v = new Vector();
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
	
	public void addCartesian(Vector v) {
		this.addCartesian(v.getX(), v.getY());
	}
	

	public void setPolar(double angle, double total) {
		this.setCartesian(Math.cos(Math.toRadians(angle)) * total, Math.sin(Math.toRadians(angle)) * total);
	}
	
	public void setAngle(double angle) {
		this.setPolar(angle, this.getMagnitude());
	}
	
	public void setTotal(double total) {
		this.setPolar(this.getAngle(), total);
	}
	
	public void addPolar(double angle, double total) {
		this.setPolar(this.getAngle() + angle, this.getMagnitude() + total);
	}
	
	public void scaleTotal(double scaleAmount) {
		this.xVal *= scaleAmount;
		this.yVal *= scaleAmount;
	}
	
	public double dot(Vector v) {
		return dot(this, v);
	}
	
	public static double dot(Vector a, Vector b) {
		return a.getX() * b.getX() + a.getY() * b.getY();
	}
	
	/**
	 * projection length of some Vector onto this
	 * @param v Vector to project onto this
	 * @return projection length of Vector v onto this
	 */
	public double projectionLengthFrom(Vector v) {
		return projectionLength(v, this);
	}
	
	/**
	 * projection length of this onto some other Vector
	 * @param v Vector to project this onto
	 * @return projection length of this onto Vector v
	 */
	public double projectionLengthOnto(Vector v) {
		return projectionLength(this, v);
	}
	
	/**
	 * projection length of Vector a onto Vector b
	 * @param a Vector to project
	 * @param b Vector projected onto
	 * @return length of the projection
	 */
	public static double projectionLength(Vector a, Vector b) {
		return dot(a, b) / b.getMagnitude();
	}
	
	public double angleBetween(Vector v) {
		return angleBetween(this, v);
	}
	
	public static double angleBetween(Vector a, Vector b) {
		double cosTheta = dot(a, b) / (a.getMagnitude() * b.getMagnitude());
		double angle = Math.toDegrees( Math.acos(cosTheta) );
		angle = ResourceFunctions.putAngleInRange(angle);

		return Math.min( angle, 360-angle );
	}
	
	public String toString() {
		return String.format("Angle: %f, Total: %f, X: %f, Y: %f",
				this.getAngle(), this.getMagnitude(), this.getX(), this.getY());
	}
	
	public static Vector normalized (Vector v)
	{
		Vector newVec = new Vector (v);
		newVec.setTotal(1.0);
		return newVec;
	}
	
}
