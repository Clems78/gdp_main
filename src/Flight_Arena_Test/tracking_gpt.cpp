#include <cmath>
#include <iostream> // Add this line to include the iostream header
using namespace std;

// struct Point {
//     double x;
//     double y;
// };

struct Coordinate 
{
    float x;
    float y;
};

// Point convertToLocalFrame(double global_x, double global_y, double drone_x, double drone_y, double drone_heading) {
//     // Calculate relative position of target in local frame
//     double relative_x = global_x - drone_x;
//     double relative_y = global_y - drone_y;

//     // Convert relative position to global frame using drone heading
//     double target_x_global = relative_x * cos(drone_heading) - relative_y * sin(drone_heading);
//     double target_y_global = relative_x * sin(drone_heading) + relative_y * cos(drone_heading);

//     // Convert to global coordinates
//     target_x_global += drone_x;
//     target_y_global += drone_y;

//     return {target_x_global, target_y_global};
// }

// Assuming the angle is provided in radians
Coordinate localToGlobal(float drone_x, float drone_y, float drone_heading, float target_local_x, float target_local_y) {
    // Rotation matrix for transforming local coordinates to global coordinates
    float cos_heading = cos(drone_heading);
    float sin_heading = sin(drone_heading);

    float target_global_x = drone_x + cos_heading * target_local_x + sin_heading * target_local_y;
    float target_global_y = drone_y + sin_heading * target_local_x - cos_heading * target_local_y;

    Coordinate target_global = {target_global_x, target_global_y};
    return target_global;
}

int main() {
    // Example usage
    double drone_global_x = 2; // Example global x coordinate of drone
    double drone_global_y = 2;  // Example global y coordinate of drone
    double drone_heading = M_PI/4; // Example heading of the drone (45 degrees in radians)
    double target_local_x = 1;  // Example local x coordinate of the target
    double target_local_y = -1.0;   // Example local y coordinate of the target

    // Convert target coordinates to global frame
    Coordinate target_global = localToGlobal(drone_global_x, drone_global_y, drone_heading, target_local_x, target_local_y);

    // Print out the result
    cout << "Target coordinates in global frame: (" << target_global.x << ", " << target_global.y << ")" << endl;

    return 0;
}
