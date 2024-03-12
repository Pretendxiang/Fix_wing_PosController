#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <utility> // pair

using namespace std;
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// hold the result 
struct NavigationResult{
    double N,E, d, delta_psi, r, L1, heta, R, phi;
    int i, F;
};

// helper function to calculate the distance between two points
double calculateDistance(double x1, double y1, double x2, double y2){
    return sqrt(pow(x2-x1,2) + pow(y2-y1,2));
}

//helper function to calcaulte the cross track error
double calculateCrossTrackError(const pair<double, double>& p1, const pair<double, double>& p2, double P_E, double P_N){
    double D  = sqrt(pow(P_N -p1.second, 2) + pow(P_E - p1.first, 2));
    double psi_r = atan2(p2.second - p1.second, p2.first - p1.first);
    double psi_c = atan2(P_N -p1.second, P_E - p1.first);
    return abs(D * sin(psi_r - psi_c));
}

// navigation function
NavigationResult GuidanceLaw(double P_E, double P_N, const vector<pair<double, double>>& path, double psi){
    const double v = 1.5;
    const int predict = 5;
    const double g = 9.81;
    NavigationResult result;
    int numWaypoints = path.size();
    double min_distance = numeric_limits<double>::max();
    int closestIndex = 1;

    for (int i = 0; i < numWaypoints; i++){
        double distance = calculateDistance(P_E, P_N, path[i].first, path[i].second);
        if (distance < min_distance){
            min_distance = distance;
            closestIndex = i;
        }
    }

    // adjusted index based on predcition horizon
    int adjustedIndex = min((closestIndex + predict), numWaypoints);

    // calculate the future position
    result.N = path[adjustedIndex].second;
    result.E = path[adjustedIndex].first;

    // calcuate delta_psi
    double psi_cmd = atan2(result.N - P_N, result.E - P_E);
    result.delta_psi = (2 * v * v / calculateDistance(P_E, P_N, result.E, result.N) * sin(psi_cmd - psi))/ v/ 0.1;
    result.phi = atan2(2 * 15 * 15 / calculateDistance(P_E, P_N, result.E, result.N) * sin(psi_cmd - psi), g);

    // convert to degrees for consistancy 
    result.phi = result.phi * 180 / M_PI;
    psi_cmd *= 180 / M_PI;

    // Cross-track error
    if (closestIndex < numWaypoints - 1) { // Ensure there is a next waypoint
        result.d = calculateCrossTrackError(path[closestIndex], path[closestIndex + 1], P_E, P_N);
    }
    
    // Simplified curvature and radius calculation (detailed calculation depends on specific requirements)
    // Here we just set some placeholder values
    result.r = 0; // Placeholder for radius of turn
    result.L1 = calculateDistance(P_E, P_N, result.N, result.E); // L1 distance
    result.heta = psi_cmd - psi; // Heading difference
    result.R = result.L1 / 2 * std::sin(result.heta); // Radius of turn, simplified
    result.F =  adjustedIndex; // Index of future waypoint
    result.i = closestIndex; // Index of closest waypoint

    return result;

}

int main(){
    // Example usage
    vector<pair<double, double>> path = {{0, 1}, {0, 2}, {0, 3}, {0, 4}, {0, 5}, {0, 6}, {0, 7}, {0, 8}, {0, 9}, {0, 10}, {0, 11}};
    
    // Example initial position and heading
    double P_E = -2, P_N = 0, psi = M_PI / 2; 
    
    NavigationResult result = GuidanceLaw(P_E, P_N, path, psi);
    
    // Example output
    cout << "Future Position (E, N): (" << result.E << ", " << result.N << ")\n";
    cout << "Future Position Index: (" << result.F << ")\n";
    cout << "Delta Psi: " << result.delta_psi << "\n";
    cout << "Delta Phi: " << result.phi << "\n";
    cout << "Cross-Track Error: " << result.d << "\n";
    cout << "Index of closest waypoint: (" << result.i << ") \n";
    return 0;
}