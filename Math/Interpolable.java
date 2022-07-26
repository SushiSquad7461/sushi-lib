package SushiFrcLib.Math;

/**
 * Credit: 2910
 */
public interface Interpolable<T> {
    T interpolate(T other, double t);
}