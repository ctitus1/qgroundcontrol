#include "CoverageGeometry.h"
#include "CenterOut.h"
#include <cassert>
#include <iostream>

int main()
{
    using namespace CoverageGeometry;
    Camera camera{1000, 1000, 1000, 1000, 500, 500};
    auto nadir = project(camera, 40, 0, -90, 0, 0);
    assert(nadir.error.isEmpty());
    assert(std::hypot(nadir.center.x(), nadir.center.y()) < 1e-10);
    assert(std::abs(nadir.spacing - 40) < 1e-10);
    auto overlap = project(camera, 40, 0, -90, 0, 75);
    assert(std::abs(overlap.spacing - 10) < 1e-10);
    Camera wide{2000, 1000, 1000, 1000, 1000, 500};
    auto lateral = project(wide, 40, 0, -90, 0, 50);
    assert(std::abs(lateral.spacing - 40) < 1e-10);
    assert(std::abs(lateral.captureDistance - 12) < 1e-10);
    auto angled = project(camera, 40, 0, -45, 20, 70);
    assert(angled.error.isEmpty() && angled.center.y() > 40);
    auto rotated = project(camera, 40, 90, -45, 110, 70);
    assert(std::abs(rotated.center.x() - angled.center.y()) < 1e-10);
    assert(std::abs(rotated.spacing - angled.spacing) < 1e-10);

    const double angle = 20 * 3.14159265358979323846 / 180;
    QPointF along(std::sin(angle), std::cos(angle)), across(along.y(), -along.x());
    for (int x : {-1, 1}) for (int y : {-1, 1}) {
        auto point = angled.center + x * angled.halfAlong * along + y * angled.halfAcross * across;
        for (int i = 0; i < 4; ++i) {
            auto edge = angled.corners[(i + 1) % 4] - angled.corners[i];
            QPointF normal(-edge.y(), edge.x());
            assert(QPointF::dotProduct(normal, point - angled.corners[i]) *
                   QPointF::dotProduct(normal, angled.center - angled.corners[i]) >= -1e-6);
        }
    }
    assert(!project(camera, 40, 0, -10, 0, 70).error.isEmpty());
    assert(!project(camera, 0, 0, -90, 0, 70).error.isEmpty());
    assert(!project(camera, 40, 0, -90, 0, 100).error.isEmpty());
    const QGeoCoordinate junction(38.31, -76.55);
    for (double rotation : {0., 90., 180., 270.}) {
        const QList<QGeoCoordinate> sector{junction, junction.atDistanceAndAzimuth(60, rotation + 90),
            junction.atDistanceAndAzimuth(100, rotation + 45), junction.atDistanceAndAzimuth(40, rotation)};
        const double inward = CenterOut::inwardHeading(sector, junction);
        assert(std::abs(inward - CenterOut::normalizeAngle(rotation + 225)) < 1e-6);
        const double trackAngle = CenterOut::normalizeAngle(inward + 90);
        assert(std::abs(CenterOut::normalizeAngle(trackAngle - inward) - 90) < 1e-6);
    }
    QList<QPair<QGeoCoordinate, QGeoCoordinate>> legs;
    for (double distance : {70., 10., 40.}) {
        const auto midpoint = junction.atDistanceAndAzimuth(distance, 0);
        legs.append({midpoint.atDistanceAndAzimuth(15, 270), midpoint.atDistanceAndAzimuth(15, 90)});
    }
    const auto passes = CenterOut::passes(legs, junction, 90);
    const auto reversed = CenterOut::passes(legs, junction, 90, true);
    assert(passes.size() == 3 && reversed.size() == passes.size());
    double previousDistance = -1, previousDirection = 0;
    for (int i = 0; i < passes.size(); ++i) {
        const auto& pass = passes[i];
        assert(pass.distance > previousDistance);
        assert(std::abs(pass.start.distanceTo(pass.end) - 30) < 0.01);
        assert(pass.start.distanceTo(reversed[i].end) < 0.01);
        const double direction = std::sin(pass.start.azimuthTo(pass.end) * M_PI / 180);
        if (i) assert(direction * previousDirection < 0);
        previousDistance = pass.distance;
        previousDirection = direction;
    }
    auto sideways = project(camera, 40, 225, -70, 315, 25);
    assert(sideways.error.isEmpty() && sideways.spacing > 0);
    camera.fx = 0;
    assert(!project(camera, 40, 0, -90, 0, 70).error.isEmpty());
    std::cout << "Coverage and center-out geometry checks passed\n";
}
