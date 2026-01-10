# PeriodicRunnable class
PeriodicRunnable is for systems that need to be run periodically, but do not need to be full subsystems.
## Decision flowchart
```mermaid
flowchart TB
    q1(Does ONE subsystem depend on/use this?) -- No --> p1(PeriodicRunnable)
    q1 -- Yes --> q2(Do multiple commands depend on this, or is it used outside of a command?)
    q2 -- Yes --> p2(PeriodicRunnable)
    q2 -- No --> s1(Subsystem)
```

## Code example
```java
class Example extends PeriodicRunnable {
  public Example() {
    super(); // Super call adds it to the registry, which calls the periodic method every tick
  }
  @Override
  public void periodic() {
    //Periodic functionality
  }
}
```
