import java.util.*;

public class NewJavaData{
  public static void main(String[] args){
    Helicopter x = new Helicopter();
    Scanner a = new Scanner(System.in);
    System.out.println(x.foo(5,6,7));
    System.out.println("Enter an amount of missles to fire");
    rockets_fired = a.nextLine();
    System.out.println(x.dumpPayload(3));


  }
}
class Helicopter extends Vehicle{
  int weight = 0;
  double operationalCapacity = 56.910;
  double payLoadRemaining = 40000;
  final double m86_weight;

  public int foo(int x, int y, float b){
    return 3;

  }
  private double dumpPayload(int m86RocketsFired){
      weight = m86RocketsFired * m86_weight;
      payLoadRemaining -= weight;
      return payLoadRemaining;
  }
}
class Vehicle{


}
