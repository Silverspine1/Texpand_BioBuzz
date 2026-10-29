package org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.Drive.Pathing.RobotUtilities;


@SuppressWarnings("unused")
public class Vector2D {
	private double x, y;

	public Vector2D(double x, double y) {
		this.x = x;
		this.y = y;
	}

	/**
	 * constructs a new default vector with values of 0 for both x and y
	 */
	public Vector2D() {
		this(0, 0);
	}


	public double getX() {
		return x;
	}

	/**
	 * mutates state
	 *
	 * @param x
	 */
	public void setX(double x) {
		this.x = x;
	}

	public double getY() {
		return y;
	}

	public double getNorm() {
		return Math.sqrt(x * x + y * y);
	}

	/**
	 * mutates state
	 *
	 * @param y
	 */
	public void setY(double y) {
		this.y = y;
	}


	public double getMagnitude() {
		return Math.hypot(x, y);
	}

	/**
	 * mutates state
	 *
	 * @param x
	 * @param y
	 * @return self
	 */
	public Vector2D set(double x, double y) {
		this.x = x;
		this.y = y;
		return this;
	}

	/**
	 * non-mutating
	 *
	 * @param x
	 * @param y
	 * @return a new vector with the desired operation applied
	 */
	public Vector2D add(double x, double y) {
		return new Vector2D(this.x + x, this.y + y);
	}

	/**
	 * non-mutating
	 *
	 * @param x
	 * @param y
	 * @return a new vector with the desired operation applied
	 */
	public Vector2D subtract(double x, double y) {
		return new Vector2D(this.x - x, this.y - y);
	}


}
