package frc.robot.util;

public class Tuple<T, V> {
  private T first;
  private V second;

  public Tuple(T first, V second) {
    this.first = first;
    this.second = second;
  }

  public T getT() {
    return first;
  }

  public V getV() {
    return second;
  }
}
