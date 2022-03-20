
#ifndef COORDINATES_H
#define COORDINATES_H

#include <iostream>

class Coordinates {
    public:
        Coordinates();
        Coordinates(double latitude, double longitude);

        double getLatitude() const;
        double getLongitude() const;

        void setLatitude(double latitude);
        void setLongitude(double longitude);

        double haversine(const Coordinates& other) const;

        friend std::ostream& operator<<(std::ostream& os, const Coordinates& obj);
    private:
        double latitude, longitude;
};

#endif // COORDINATES_H