#pragma once
#include "SurveyComplexItem.h"

class PlannerSurvey : public SurveyComplexItem
{
public:
    using SurveyComplexItem::SurveyComplexItem;
    QList<QPair<QGeoCoordinate, QGeoCoordinate>> legs() const {
        QList<QPair<QGeoCoordinate, QGeoCoordinate>> result;
        for (const auto& transect : _transects) {
            QGeoCoordinate entry;
            for (const auto& point : transect) {
                if (point.coordType == CoordTypeSurveyEntry) entry = point.coord;
                if (point.coordType == CoordTypeSurveyExit && entry.isValid()) result.append({entry, point.coord});
            }
        }
        return result;
    }
};
