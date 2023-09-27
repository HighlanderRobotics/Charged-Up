package frc.lib.choreolib;

public enum ChoreoPathFeature {
  none {
    public String getLocalizedDescription() {
      return "None";
    }
  },
  park {
    public String getLocalizedDescription() {
      return "Park";
    }
  },
  mobility {
    public String getLocalizedDescription() {
      return "Mobility";
    }
  },
  balance {
    public String getLocalizedDescription() {
      return "Balance";
    }
  };

  public abstract String getLocalizedDescription();
}
