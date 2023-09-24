package frc.lib.choreolib;

import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class ChoreoPath {
    public int pieceCount;
    public AutoFieldPosition fieldPosition;
    public ChoreoPathFeature feature;

    public ChoreoPath(int pieceCount,
                      AutoFieldPosition fieldPosition,
                      ChoreoPathFeature feature
    ) {
        this.pieceCount = pieceCount;
        this.fieldPosition = fieldPosition;
        this.feature = feature;
        // this.feature = feature;
    }

    // TODO: Add FileName to Path with Error Handling
    public ChoreoPath(String fileName) {
    }

    public String localizedDescription() {
        return pieceCount +
                " " +
                (feature != ChoreoPathFeature.none ? "+ " + feature.getLocalizedDescription() : "") + 
                "Blue" +
                " " +
                (fieldPosition == AutoFieldPosition.Clear ? "Clear" : "Bump");
    }

    public String fileName() {
        return localizedDescription() + ".json";
    }
}