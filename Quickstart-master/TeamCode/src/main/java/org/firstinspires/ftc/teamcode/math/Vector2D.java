package org.firstinspires.ftc.teamcode.math;

public class Vector2D {
    public double x;
    public double y;

    // Constructor
    public Vector2D(double x, double y) {
        this.x = x;
        this.y = y;
    }

    // Zero vector
    public static Vector2D zero() {
        return new Vector2D(0, 0);
    }

    // Add
    public Vector2D plus(Vector2D other) {
        return new Vector2D(this.x + other.x, this.y + other.y);
    }

    // Subtract
    public Vector2D minus(Vector2D other) {
        return new Vector2D(this.x - other.x, this.y - other.y);
    }

    // Scale
    public Vector2D times(double scalar) {
        return new Vector2D(this.x * scalar, this.y * scalar);
    }

    // Divide
    public Vector2D div(double scalar) {
        return new Vector2D(this.x / scalar, this.y / scalar);
    }

    // Dot product
    public double dot(Vector2D other) {
        return this.x * other.x + this.y * other.y;
    }

    //Cross product
    public double cross(Vector2D other) {
        return this.x * other.y - this.y * other.x;
    }

    // Magnitude
    public double mag() {
        return Math.hypot(x, y);
    }

    // Normalized vector
    public Vector2D normalized() {
        double mag = this.mag();
        if (mag == 0) {
            return Vector2D.zero();
        }
        else return this.div(mag);
    }

    // Rotate by angle (radians)
    public Vector2D rotated(double angle) {
        double cos = Math.cos(angle);
        double sin = Math.sin(angle);
        return new Vector2D(x * cos - y * sin, x * sin + y * cos);
    }

    // Angle of vector (radians)
    public double get_angle() {
        return Math.atan2(y, x);
    }

    @Override
    public String toString() {
        return String.format("(%.3f, %.3f)", x, y);
    }

    public double dist(Vector2D other){
        return Math.hypot(this.x-other.x, this.y-other.y);
    }
}