package org.firstinspires.ftc.teamcode.config.Limelight;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

/**
 * AprilTag Distance Detection using Limelight 3A
 * Detects a specific AprilTag and calculates distance
 * No 3D localization required - uses area-based calculation
 */
public class AprilTagDistance {

    private Limelight3A limelight;
    private int targetAprilTagID;

    // Distance calculation constants (calibrate these for your setup)
    private static final double DISTANCE_CONSTANT = 161.1;
    private static final double DISTANCE_EXPONENT = -0.5858;

    /**
     * Constructor for AprilTagDistance
     * @param hwMap Hardware map from OpMode
     * @param targetID The AprilTag ID you want to track (e.g., 24)
     */
    public AprilTagDistance(HardwareMap hwMap, int targetID) {
        limelight = hwMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(0); // Switch to your AprilTag pipeline
        limelight.start();

        this.targetAprilTagID = targetID;
    }

    /**
     * Get distance to the target AprilTag in centimeters
     * Uses area-based calculation (no 3D localization needed)
     * @return Distance in CM, or -1 if tag not found
     */
    public double getDistanceCM() {
        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid()) {
            return -1;
        }

        List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();

        if (fiducialResults.isEmpty()) {
            return -1;
        }

        // Find the target AprilTag
        for (LLResultTypes.FiducialResult fr : fiducialResults) {
            if (fr.getFiducialId() == targetAprilTagID) {
                double ta = result.getTa(); // Target area percentage

                if (ta > 0) {
                    // Area-based distance formula
                    return DISTANCE_CONSTANT * Math.pow(ta, DISTANCE_EXPONENT);
                }
            }
        }

        return -1; // Target tag not found
    }

    /**
     * Get distance to ANY visible AprilTag (doesn't filter by ID)
     * @return Distance in CM, or -1 if no tag found
     */
    public double getDistanceToAnyTag() {
        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid()) {
            return -1;
        }

        double ta = result.getTa(); // Target area percentage

        if (ta > 0) {
            return DISTANCE_CONSTANT * Math.pow(ta, DISTANCE_EXPONENT);
        }

        return -1;
    }

    /**
     * Get the ID of the currently detected AprilTag
     * @return AprilTag ID, or -1 if none found
     */
    public int getDetectedAprilTagID() {
        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid()) {
            return -1;
        }

        List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();

        if (!fiducialResults.isEmpty()) {
            return fiducialResults.get(0).getFiducialId();
        }

        return -1;
    }

    /**
     * Check if the target AprilTag is currently visible
     * @return true if target tag is detected, false otherwise
     */
    public boolean isTargetVisible() {
        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid()) {
            return false;
        }

        List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();

        for (LLResultTypes.FiducialResult fr : fiducialResults) {
            if (fr.getFiducialId() == targetAprilTagID) {
                return true;
            }
        }

        return false;
    }

    /**
     * Check if ANY AprilTag is visible
     * @return true if any tag is detected, false otherwise
     */
    public boolean isAnyTagVisible() {
        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid()) {
            return false;
        }

        List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
        return !fiducialResults.isEmpty();
    }

    /**
     * Get horizontal offset (tx) from target AprilTag
     * Useful for alignment
     * @return Horizontal offset in degrees, or 0 if not found
     */
    public double getHorizontalOffset() {
        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid()) {
            return 0;
        }

        List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();

        for (LLResultTypes.FiducialResult fr : fiducialResults) {
            if (fr.getFiducialId() == targetAprilTagID) {
                return result.getTx();
            }
        }

        return 0;
    }

    /**
     * Get vertical offset (ty) from target AprilTag
     * @return Vertical offset in degrees, or 0 if not found
     */
    public double getVerticalOffset() {
        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid()) {
            return 0;
        }

        List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();

        for (LLResultTypes.FiducialResult fr : fiducialResults) {
            if (fr.getFiducialId() == targetAprilTagID) {
                return result.getTy();
            }
        }

        return 0;
    }

    /**
     * Get target area percentage
     * Useful for debugging
     * @return Target area (0-100%), or -1 if not found
     */
    public double getTargetArea() {
        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid()) {
            return -1;
        }

        return result.getTa();
    }

    /**
     * Get all visible AprilTag IDs
     * @return List of all detected AprilTag IDs
     */
    public List<Integer> getAllVisibleTagIDs() {
        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid()) {
            return null;
        }

        List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
        List<Integer> tagIDs = new java.util.ArrayList<>();

        for (LLResultTypes.FiducialResult fr : fiducialResults) {
            tagIDs.add((int) fr.getFiducialId());
        }

        return tagIDs;
    }

    /**
     * Change the target AprilTag ID during runtime
     * @param newTargetID The new AprilTag ID to track
     */
    public void setTargetAprilTagID(int newTargetID) {
        this.targetAprilTagID = newTargetID;
    }

    /**
     * Get the current target AprilTag ID
     * @return The target AprilTag ID
     */
    public int getTargetAprilTagID() {
        return targetAprilTagID;
    }

    /**
     * Switch Limelight pipeline
     * @param pipelineIndex Pipeline index (0-9)
     */
    public void switchPipeline(int pipelineIndex) {
        limelight.pipelineSwitch(pipelineIndex);
    }

    /**
     * Stop the Limelight
     */
    public void stop() {
        limelight.stop();
    }
}