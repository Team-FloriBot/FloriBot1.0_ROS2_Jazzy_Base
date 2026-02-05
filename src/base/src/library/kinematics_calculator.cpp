#include "base/kinematics_calculator.h"

namespace base {

KinematicsCalculator::KinematicsCalculator(double wheel_separation, double wheel_radius)
    : wheel_sep_(wheel_separation), wheel_rad_(wheel_radius) {}
/**
 * Erweiterte Berechnung mit Schlupfkompensation (basierend auf PDF Lösung B)
 * @param linear_x: Zielgeschwindigkeit Roboter
 * @param angular_z: Ziel-Drehrate Roboter
 * @param actual_twist: Die REALE Bewegung (vom LiDAR gemessen!)
 */
WheelSpeedSet KinematicsCalculator::calculateWheelSpeeds(
    double linear_x, 
    double angular_z, 
    const RobotTwist& actual_twist) 
{
    // 1. Berechne die theoretischen Radgeschwindigkeiten (Soll)
    double v_left_d = linear_x - (angular_z * wheel_sep_ / 2.0);
    double v_right_d = linear_x + (angular_z * wheel_sep_ / 2.0);

    // 2. Schlupfschätzung (Der Kern von Lösung B aus dem PDF)
    // Wir vergleichen, was wir wollten (linear_x, angular_z) 
    // mit dem, was der LiDAR misst (actual_twist).
    
    // Berechne die reale Geschwindigkeit an jedem Rad laut LiDAR
    double v_left_act = actual_twist.linear_x - (actual_twist.angular_z * wheel_sep_ / 2.0);
    double v_right_act = actual_twist.linear_x + (actual_twist.angular_z * wheel_sep_ / 2.0);

    // Längsschlupf-Kompensationsterm v_S: Differenz zwischen Soll und Ist
    double k_slip = 0.7; // Schlupf wird nicht vollständig kompoensiert
    // bei vollständiger Kompensation kann es passieren, dass das Fahrverhalten zu nervös wird
    double v_s_left = (v_left_d - v_left_act) * k_slip;
    double v_s_right = (v_right_d - v_right_act) * k_slip;

    // 3. Korrigierte Winkelgeschwindigkeit berechnen
    // Formel aus PDF: omega_corrected = (v_desired + v_slip) / radius
    // Wir addieren den Schlupf als Korrektur hinzu
    double rad_s_left = (v_left_d + v_s_left) / wheel_rad_;
    double rad_s_right = (v_right_d + v_s_right) / wheel_rad_;

    return {rad_s_left, rad_s_right};
}

RobotTwist KinematicsCalculator::calculateRobotTwist(const WheelSpeedSet& speeds) {
    // Diese Funktion bleibt gleich, da sie die "Encoder-Odometrie" berechnet
    double vel_left = speeds.left * wheel_rad_;
    double vel_right = speeds.right * wheel_rad_;

    RobotTwist twist;
    twist.linear_x = (vel_left + vel_right) / 2.0;
    twist.linear_y = 0.0;
    twist.angular_z = (vel_right - vel_left) / wheel_sep_;
    
    return twist;
}

} 
