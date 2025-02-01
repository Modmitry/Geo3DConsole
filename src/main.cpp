#include "Geometry.h"

// User input for a point
//-------------------------------------------------------------------------
P3D getPointFromUser()
{
    static int pointCount = 1;
    double x{ 0.0 }, y{ 0.0 }, z{ 0.0 };

    while (true) 
    {
        std::cout << "Enter coordinates for point " << pointCount << " (x y z): \n";
        if (std::cin >> x >> y >> z) 
        {
            pointCount++;
            return P3D(x, y, z);
        }

        std::cerr << "Invalid input. Please enter numeric values.\n";
        std::cin.clear();
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    }
}

// Output the minimal distance result
//-------------------------------------------------------------------------
void displayMinimalDistance(const double distance)
{
    std::cout << "Minimal distance: " << distance << '\n';
}
//-------------------------------------------------------------------------

int main()
{
    // Get coordinates for all points
	const P3D p1 = getPointFromUser();
	const P3D p2 = getPointFromUser();
	const P3D p3 = getPointFromUser();
	const P3D p4 = getPointFromUser();

    // Calculate the minimal distance between the segments
    DistanceCalculator distanceCalculator(p1, p2, p3, p4);
    const auto dist=distanceCalculator.calculate();
    displayMinimalDistance(dist);

    return 0;
}