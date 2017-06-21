//-------------------------------------------------------------------------------------------------
// File : photonmap.h
// Desc : Photon map data structure.
// Copyright(c) Project Asura. All right reserved.
//-------------------------------------------------------------------------------------------------
#pragma once

//-------------------------------------------------------------------------------------------------
// Includes
//-------------------------------------------------------------------------------------------------
#include <r3d_math.h>
#include <cstdint>


///////////////////////////////////////////////////////////////////////////////////////////////////
// Photon structure
///////////////////////////////////////////////////////////////////////////////////////////////////
struct Photon
{
    Vector3     pos;
    int16_t     plane;
    uint8_t     theta, phi;
    Vector3     power;
};

///////////////////////////////////////////////////////////////////////////////////////////////////
// NearestPhotons structure
///////////////////////////////////////////////////////////////////////////////////////////////////
struct NearestPhotons
{
    int             max;
    int             found;
    int             got_heap;
    Vector3         pos;
    double*         dist2;
    const Photon**  index;
};

///////////////////////////////////////////////////////////////////////////////////////////////////
// PhotonMap class
///////////////////////////////////////////////////////////////////////////////////////////////////
class PhotonMap
{
    //=============================================================================================
    // list of friend classes and methods.
    //=============================================================================================
    /* NOTHING */

public:
    //=============================================================================================
    // public variables.
    //=============================================================================================
    /* NOTHING */

    //=============================================================================================
    // public methods.
    //=============================================================================================

    PhotonMap(int max_photons);

    ~PhotonMap();

    void store(const Vector3& power, const Vector3& pos, const Vector3& dir);

    void scale_photon_power(const double scale);

    void balance();

    void irradiance_estimate(
        Vector3& irradiance,
        const Vector3 pos,
        const Vector3 normal,
        const double max_dist,
        const int photon_count);

    void locate_photons(NearestPhotons* const nearest_photon, const int index) const;

    void photon_dir(Vector3& dir, const Photon* photon) const;

private:
    void balance_segment(
        Photon** pbal,
        Photon** porg,
        const int index,
        const int start,
        const int end);

    void median_split(
        Photon** p,
        const int start,
        const int end,
        const int median,
        const int axis);

    Photon* photons;
    int stored_photons;
    int half_stored_photons;
    int max_photons;
    int prev_scale;

    double costheta[256];
    double sintheta[256];
    double cosphi[256];
    double sinphi[256];

    Vector3 bbox_min;
    Vector3 bbox_max;
};