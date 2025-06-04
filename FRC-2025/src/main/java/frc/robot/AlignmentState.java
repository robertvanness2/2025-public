package frc.robot;

public class AlignmentState {
  public enum ReefStackChoice {
    LEFT,
    RIGHT;
  }

  public enum ReefLevel {
    ONE,
    TWO,
    THREE,
    FOUR
  }

  private static AlignmentState m_instance;

  public static AlignmentState getInstance() {
    if (m_instance == null) {
      m_instance = new AlignmentState();
    }
    return m_instance;
  }

  private ReefStackChoice m_selectedReefStackChoice = ReefStackChoice.RIGHT;
  private ReefLevel m_selectedReefLevel = ReefLevel.ONE;

  public ReefStackChoice getStackChoice() {
    return m_selectedReefStackChoice;
  }

  public ReefLevel getScoringLevel() {
    return m_selectedReefLevel;
  }

  public void setStackChoice(ReefStackChoice choice) {
    m_selectedReefStackChoice = choice;
  }

  public void setScoringLevel(ReefLevel level) {
    m_selectedReefLevel = level;
  }
}
