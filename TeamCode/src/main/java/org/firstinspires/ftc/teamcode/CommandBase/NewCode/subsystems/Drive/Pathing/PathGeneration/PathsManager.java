package org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.Drive.Pathing.PathGeneration;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;
import java.util.Objects;

import org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.Drive.Pathing.PathGeneration.commands.SectionBuilder;
import org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.Drive.Pathing.PathingUtility.PathingVelocity;
import org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.Drive.Pathing.RobotUtilities.RobotConfig;
import org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.Drive.Pathing.RobotUtilities.Vector2D;

public class PathsManager {

    private Map<String, PathBuilder> objectMap = new HashMap<>();

    public ArrayList<String> paths = new ArrayList<>();

    String currentPath = null;

    RobotConfig robotConfig = new RobotConfig();

    public PathsManager(){}

    public PathsManager(RobotConfig CustomConfig){
        robotConfig = CustomConfig;
    }

    public void addNewPath(String pathName){

        PathBuilder newBuilder = new PathBuilder(robotConfig);

        objectMap.put(pathName, newBuilder);

        paths.add(pathName);

        currentPath = pathName;
    }

    public void addNewPath(String pathName, SectionBuilder[] pathSections) {

        PathBuilder newBuilder = new PathBuilder(robotConfig);

        objectMap.put(pathName, newBuilder);

        paths.add(pathName);

        currentPath = pathName;

        for (String s : paths) {
            if (s.equals(pathName)) {
                currentPath = pathName;

                if (robotConfig.logDebugging()){
                    System.out.println("");
                    System.out.println("trying: " + currentPath);
                }

                Objects.requireNonNull(objectMap.get(currentPath)).buildPath(pathSections);

                if (robotConfig.logDebugging()){
                    System.out.println("built: " + currentPath);
                }

                break;
            }
        }
    }

    public void buildPath(SectionBuilder[] pathSections, String pathName){
        for (String s : paths) {
            if (s.equals(pathName)) {
                currentPath = pathName;

                if (robotConfig.logDebugging()){
                    System.out.println("");
                    System.out.println("trying: " + currentPath);
                }

                Objects.requireNonNull(objectMap.get(currentPath)).buildPath(pathSections);

                if (robotConfig.logDebugging()){
                    System.out.println("built: " + currentPath);
                }

                break;
            }
        }
    }

    public void buildPath(SectionBuilder[] pathSections, String pathName, double newAccelMax){
        for (String s : paths) {
            if (s.equals(pathName)) {
                currentPath = pathName;

                if (robotConfig.logDebugging()){
                    System.out.println("");
                    System.out.println("trying: " + currentPath);
                }

                Objects.requireNonNull(objectMap.get(currentPath)).buildPath(pathSections, newAccelMax);

                if (robotConfig.logDebugging()){
                    System.out.println("built: " + currentPath);
                }

                break;
            }
        }
    }

    public void buildPath(SectionBuilder[] pathSections){

        if (currentPath != null) {

            if (robotConfig.logDebugging()){
                System.out.println("");
                System.out.println("trying: " + currentPath);
            }

            Objects.requireNonNull(objectMap.get(currentPath)).buildPath(pathSections);

            if (robotConfig.logDebugging()){
                System.out.println("built: " + currentPath);
            }

        }

    }

    public void buildPath(SectionBuilder[] pathSections, double newAccelMax){

        if (currentPath != null) {

            if (robotConfig.logDebugging()){
                System.out.println("");
                System.out.println("trying: " + currentPath);
            }

            Objects.requireNonNull(objectMap.get(currentPath)).buildPath(pathSections, newAccelMax);

            if (robotConfig.logDebugging()){
                System.out.println("built: " + currentPath);
            }

        }

    }

    public void addPoints(Vector2D start, Vector2D end){
        Objects.requireNonNull(objectMap.get(currentPath)).buildPathSegment(start, end);
    }

    public void addPoints(Vector2D start, Vector2D control, Vector2D end){
        Objects.requireNonNull(objectMap.get(currentPath)).buildPathSegment(start, control, end);
    }

    public void addPoints(Vector2D start, Vector2D control1, Vector2D control2, Vector2D end){
            Objects.requireNonNull(objectMap.get(currentPath)).buildPathSegment(start, control1, control2, end);
    }

    public String getCurrentPath() {
        return currentPath;
    }

    public void setCurrentPath(String currentPath) {
        for (String s : paths) {
            if (s.equals(currentPath)) {
                this.currentPath = currentPath;
                break;
            }
        }
    }

    public ArrayList<Vector2D> returnPathingPoints(){
        return Objects.requireNonNull(objectMap.get(currentPath)).followablePath;
    }

    public ArrayList<Vector2D> returnPathingPoints(String pathName){
        currentPath = pathName;
        return Objects.requireNonNull(objectMap.get(currentPath)).followablePath;
    }

    public ArrayList<PathingVelocity> returnVectorField(){
        return Objects.requireNonNull(objectMap.get(currentPath)).pathingVelocity;
    }

    public ArrayList<PathingVelocity> returnVectorField(String pathName){
        currentPath = pathName;
        return Objects.requireNonNull(objectMap.get(currentPath)).pathingVelocity;
    }

    public PathBuilder returnPath(String pathName){
        return Objects.requireNonNull(objectMap.get(pathName));
    }

    public PathBuilder returnCurrentPath(){
        return Objects.requireNonNull(objectMap.get(currentPath));
    }


}
