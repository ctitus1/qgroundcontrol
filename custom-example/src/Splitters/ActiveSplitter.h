/****************************************************************************
 *
 * (c) 2009-2024 QGROUNDCONTROL PROJECT <http://www.qgroundcontrol.org>
 *
 * QGroundControl is licensed according to the terms in the file
 * COPYING.md in the root of the source code directory.
 *
 ****************************************************************************/

#pragma once

/*
 * Compile-time selection of the active region splitter.
 *
 * To try a different splitting strategy, add a new RegionSplitter subclass to
 * this folder and change the two lines below to point at it. Nothing else in
 * the plugin needs to change (the manager only ever talks to the abstract
 * RegionSplitter interface via ActiveRegionSplitter).
 */

#include "ControlPointSplitter.h"

using ActiveRegionSplitter = ControlPointSplitter;
