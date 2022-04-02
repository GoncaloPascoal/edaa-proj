
#include <cmath>
#include <unordered_set>
#include "ordered_quadtree.hpp"

using namespace std;

AABB::AABB(Coordinates topLeft, Coordinates bottomRight) : topLeft(topLeft),
    bottomRight(bottomRight) {}

Coordinates AABB::center() const {
    double radiusLat = abs(topLeft.getLatitude() - bottomRight.getLatitude()) / 2,
        radiusLong = abs(topLeft.getLongitude() - bottomRight.getLongitude()) / 2;

    return Coordinates(topLeft.getLatitude() + radiusLat, topLeft.getLongitude() + radiusLong);
}

double AABB::maxDimension() const {
    return max(
        abs(topLeft.getLatitude() - bottomRight.getLatitude()),
        abs(topLeft.getLongitude() - bottomRight.getLongitude())
    );
}

bool AABB::containsPoint(const Coordinates& coords) const {
    return coords.getLatitude() >= topLeft.getLatitude() &&
        coords.getLatitude() <= bottomRight.getLatitude() &&
        coords.getLongitude() >= topLeft.getLongitude() &&
        coords.getLongitude() <= bottomRight.getLongitude();
}

bool AABB::quadIntersects(const Coordinates& center, double radius) const {
    double cLat = center.getLatitude(), cLong = center.getLongitude(),
        minLat = topLeft.getLatitude(), maxLat = bottomRight.getLatitude(),
        minLong = topLeft.getLongitude(), maxLong = bottomRight.getLongitude();

    return maxLat >= cLat - radius && minLat <= cLat + radius &&
        maxLong >= cLong - radius && minLong <= cLong + radius;
}

array<AABB, 4> AABB::split() const {
    double tLat = topLeft.getLatitude(), tLong = topLeft.getLongitude(),
        bLat = bottomRight.getLatitude(), bLong = bottomRight.getLongitude();

    double deltaLat = abs(tLat - bLat), deltaLong = abs(tLong - bLong);

    Coordinates topMid(tLat + deltaLat / 2, tLong),
            midLeft(tLat, tLong + deltaLong / 2),
            center(tLat + deltaLat / 2, tLong + deltaLong / 2),
            midRight(tLat + deltaLat, tLong + deltaLong / 2),
            bottomMid(tLat + deltaLat / 2, tLong + deltaLong);

    return {
        AABB(topLeft, center),
        AABB(topMid, midRight),
        AABB(midLeft, bottomMid),
        AABB(center, bottomRight)
    };
}

ostream& operator<<(ostream& os, const AABB& obj) {
    os << obj.topLeft << " - " << obj.bottomRight;
    return os;
}

OrderedQuadtree::OrderedQuadtree(AABB boundary) : boundary(boundary), point(nullptr) {}

OrderedQuadtree::~OrderedQuadtree() {
    delete nw;
    delete ne;
    delete sw;
    delete se;
}

void OrderedQuadtree::insert(const OsmNode& newPoint) {
    auxiliarList.insert(newPoint);
    insertRec(newPoint);
}

void OrderedQuadtree::insertRec(const OsmNode& newPoint) {
    if (!boundary.containsPoint(newPoint.coordinates)) {
        return;
    }

    if (nw == nullptr) {
        // Leaf node
        if (point == nullptr) {
            point = &newPoint;
            return;
        }
        // Doesn't have space
        this->subdivide();
    }

    nw->insertRec(newPoint);
    ne->insertRec(newPoint);
    sw->insertRec(newPoint);
    se->insertRec(newPoint);
}

void OrderedQuadtree::subdivide() {
    array<AABB, 4> newBoundaries = boundary.split();

    nw = new OrderedQuadtree(newBoundaries[0]);
    ne = new OrderedQuadtree(newBoundaries[1]);
    sw = new OrderedQuadtree(newBoundaries[2]);
    se = new OrderedQuadtree(newBoundaries[3]);

    nw->insertRec(*point);
    ne->insertRec(*point);
    sw->insertRec(*point);
    se->insertRec(*point);

    point = nullptr;
}

const OsmNode* OrderedQuadtree::nearestNeighbor(const Coordinates& queryPoint) const {
    NNResult best;
    best.distance = 2 * boundary.maxDimension();
    findNearest(queryPoint, best);

    OsmNode lowerBound;
    lowerBound.coordinates = Coordinates(queryPoint.getLatitude(), queryPoint.getLongitude() - best.distance);
    auto itr = auxiliarList.lower_bound(lowerBound);

    for (; itr != auxiliarList.end(); itr++) {
        if (queryPoint.euclideanDistance(itr->coordinates) < best.distance) {
            // We found a closer point to queryPoint
            best.point = &(*itr);
            best.distance = queryPoint.euclideanDistance(itr->coordinates);
        }

        if (queryPoint.getLongitude() + best.distance > itr->coordinates.getLongitude()) {
            break;
        }
    }

    return best.point;
}

ostream& operator<<(ostream& os, const OrderedQuadtree& obj) {
    if (obj.nw) {
        os << "NW [" << *(obj.nw) << "] ";
    }

    if (obj.ne) {
        os << "NE [" << *(obj.ne) << "] ";
    }

    if (obj.point == nullptr) {
        os << "null";
    }
    else {
        os << obj.point->coordinates;
    }
    os << " -> " << obj.boundary;

    if (obj.sw) {
        os << " SW [" << *(obj.sw) << "] ";
    }

    if (obj.se) {
        os << "SE [" << *(obj.se) << "]";
    }

    return os;
}

const OrderedQuadtree* OrderedQuadtree::selectQuadrant(const Coordinates& queryPoint) const {
    if (nw == nullptr) {
        return this;
    }

    Coordinates center = boundary.center();

    if (queryPoint.getLatitude() <= center.getLatitude()) {
        if (queryPoint.getLongitude() <= center.getLongitude())
            return nw;
        else
            return sw;
    }
    else {
        if (queryPoint.getLongitude() <= center.getLongitude())
            return ne;
        else
            return se;
    }

    // Should never be reached
    return nullptr;
}

bool OrderedQuadtree::findNearest(const Coordinates& queryPoint, NNResult& best) const {
    if (point == nullptr && nw == nullptr) {
        return false;
    }

    if (point) {
        best.point = point;
        best.distance = queryPoint.euclideanDistance(point->coordinates);
        return true;
    }

    if (nw != nullptr) {
        // Search the most likely child first, then the other three
        const OrderedQuadtree* next = selectQuadrant(queryPoint);

        if (next->findNearest(queryPoint, best)) {
            return true;
        };

        for (auto child : {nw, ne, sw, se}) {
            if (child != next) {
                if (child->findNearest(queryPoint, best)) {
                    return true;
                }
            }
        }
        return false;
    }
}
