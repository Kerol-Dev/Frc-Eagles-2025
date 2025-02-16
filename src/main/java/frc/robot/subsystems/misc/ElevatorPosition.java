package frc.robot.subsystems.misc;

public enum ElevatorPosition {
    idle(0.0),
    grab_algae_reef_1(1.0),
    grab_algae_reef_2(2.0),
    grab_coral_source(3.0),
    place_algae_processor(4.0),
    place_coral_l1(5.0),
    place_coral_l2(6.0),
    place_coral_l3(7.0),
    place_coral_l4(8.0);

    private final double position;

    ElevatorPosition(double position) {
        this.position = position;
    }

    public double getPosition() {
        return position;
    }
}