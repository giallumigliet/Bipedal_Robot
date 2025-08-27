#include <iostream>
#include <cmath>
#include <algorithm>


struct Angles {
    double gamma; // angolo rotazione piano YZ
    double theta; // angolo femore
    double phi;   // angolo ginocchio
    bool valid;   // true se la soluzione è valida
};

// Funzione di clipping [-1,1] per sicurezza numerica
inline double clamp(double x, double minVal = -1.0, double maxVal = 1.0) {
    return std::max(minVal, std::min(maxVal, x));
}

Angles inverseKinematics(double X, double Y, double Z, double A, double E, double F) {
    // --- Piano YZ ---
    double C2 = Y*Y + Z*Z;
    if (C2 < A*A) {
        return {0,0,0,false}; // punto non raggiungibile
    }

    double C = std::sqrt(C2);
    double D = std::sqrt(C*C - A*A);
    double delta = std::atan2(Y, Z);
    double epsilon = std::atan2(D, A);
    double omega = delta + epsilon;
    double gamma = omega - M_PI/2.0;

    // Proiezioni
    double Dz = D * std::cos(gamma);
    double Ez = E * std::cos(gamma);
    double Fz = F * std::cos(gamma);

    // --- Piano XZ ---
    double G = std::sqrt(Dz*Dz + X*X);

    // Controllo raggiungibilità (legge dei coseni)
    double cosPhi = (G*G - Ez*Ez - Fz*Fz) / (-2.0*Ez*Fz);
    cosPhi = clamp(cosPhi);

    double phi = std::acos(cosPhi);

    double alpha = std::atan2(X, Dz);
    double sinArg = (Fz * std::sin(phi)) / G;
    sinArg = clamp(sinArg);

    double beta  = std::asin(sinArg);
    double theta = alpha + beta;

    return {gamma, theta, phi, true};
}

int main() {
    // Costanti note (esempio)
    double A = 5.0;   // cm
    double E = 10.0;  // cm
    double F = 10.0;  // cm

    // Posizione desiderata del piede
    double X = 8.0, Y = 5.0, Z = -12.0;

    Angles sol = inverseKinematics(X, Y, Z, A, E, F);

    if (sol.valid) {
        std::cout << "Gamma = " << sol.gamma * 180.0/M_PI << " deg\n";
        std::cout << "Theta = " << sol.theta * 180.0/M_PI << " deg\n";
        std::cout << "Phi   = " << sol.phi   * 180.0/M_PI << " deg\n";
    } else {
        std::cout << "Posizione non raggiungibile!\n";
    }

    return 0;
}


