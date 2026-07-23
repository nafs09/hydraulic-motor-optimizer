#pragma once
// dxf_export.h
// Replaced: the original implementation wrote POINT entities to a .dxf file
// using a raw CP array lookup, which SolidWorks could not import as a curve.
//
// Now writes a SolidWorks "Curve Through XYZ Points" (.sldcrv) file instead.
// The namespace and function signature are unchanged so no other file needs
// editing except main.cpp (which never called export_cam — see below).
//
// Output file location
// --------------------
// Files are written to the process working directory — the same place the
// binary runs from (typically the build output folder, e.g. build/Release/).
// The returned string is the full path that was written, or "" on failure.
//
// SolidWorks import
// -----------------
// Insert → Curve → Curve Through XYZ Points → browse to cam_profile.sldcrv
// Units are millimetres. The profile is a closed loop (last point = first).

#include "design.h"
#include <string>

namespace dxf_export {

    // Write cam_profile.sldcrv (or the given filename) to the working directory.
    // Uses config::DXF_N_POINTS sample points and config::DXF_SCALE (1000 = m→mm).
    // Returns the path written, or "" if the file could not be opened.
    std::string export_cam(const Design& d,
                           const std::string& filename = "cam_profile.sldcrv");

} // namespace dxf_export
