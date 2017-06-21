//------------------------------------------------------------------------------------------------
// File : photonmap.cpp
// Desc : Photon Map Data structure.
// Copyright(c) Project Asura. All right reserved.
//-------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------
// Includes
//-------------------------------------------------------------------------------------------------
#include "photonmap.h"
#include <cstdlib>
#include <malloc.h>


namespace {

inline void swap(Photon** ph, int a, int b)
{
    Photon* ph2 = ph[a];
    ph[a] = ph[b];
    ph[b] = ph2;
}

} // namespace


///////////////////////////////////////////////////////////////////////////////////////////////////
// PhotonMap
///////////////////////////////////////////////////////////////////////////////////////////////////

//-------------------------------------------------------------------------------------------------
// This is the constructor for the photon map.
// To create the photon map it is necessary to specify the maximum number of photons that will be stored.
//-------------------------------------------------------------------------------------------------
PhotonMap::PhotonMap(int max_count)
{
    stored_photons = 0;
    prev_scale = 1;
    max_photons = max_count;

    photons = static_cast<Photon*>(malloc(sizeof(Photon) * (max_photons + 1)));
    if (photons == nullptr)
    {
        abort();
    }

    bbox_min.x = bbox_min.y = bbox_min.z =  1e8;
    bbox_max.x = bbox_max.y = bbox_max.z = -1e8;

    // initialize direction conversion tables
    for (auto i = 0; i < 256; ++i)
    {
        double angle = double(i) * (1.0 / 256.0) * D_PI;
        costheta[i] = cos(angle);
        sintheta[i] = sin(angle);
        cosphi[i] = cos(2.0 * angle);
        sinphi[i] = sin(2.0 * angle);
    }
}

//-------------------------------------------------------------------------------------------------
// destructor.
//-------------------------------------------------------------------------------------------------
PhotonMap::~PhotonMap()
{
    if (photons != nullptr)
    {
        free(photons);
        photons = nullptr;
    }
}

//-------------------------------------------------------------------------------------------------
// photon_dir returns the direciton of photon.
//-------------------------------------------------------------------------------------------------
void PhotonMap::photon_dir(Vector3& dir, const Photon* p) const
{
    dir.x = sintheta[p->theta] * cosphi[p->phi];
    dir.y = sintheta[p->theta] * sinphi[p->phi];
    dir.z = costheta[p->theta];
}

//-------------------------------------------------------------------------------------------------
// irradiance_estimate computes an irradiance estimate at a given surface position.
//-------------------------------------------------------------------------------------------------
void PhotonMap::irradiance_estimate
(
    Vector3&        irrad,      // returned irradiance.
    const Vector3   pos,        // surface position.
    const Vector3   normal,     // surface normal at pos.
    const double    max_dist,   // max distance to look for photons.
    const int       nphotons    // number of photons to use.
)
{
    irrad.x = irrad.y = irrad.z = 0.0;

    NearestPhotons np;
    np.dist2    = static_cast<double*>(alloca(sizeof(double) * (nphotons + 1)));            // スタックメモリからアロケートするのでfree()する必要なし.
    np.index    = static_cast<const Photon**>(alloca(sizeof(Photon*) * (nphotons + 1)));    // スタックメモリからアロケートするのでfree()する必要なし.
    np.pos      = pos;
    np.max      = nphotons;
    np.found    = 0;
    np.got_heap = 0;
    np.dist2[0] = max_dist * max_dist;

    // locate the nearest photons
    locate_photons(&np, 1);

    // if less than 8 photons return
    if (np.found < 8)
    { return; }

    Vector3 pdir;

    // sum irradiance from all photons
    for (auto i = 1; i <= np.found; ++i)
    {
        const Photon* p = np.index[i];

        // the photon_dir call and following if can omitted (for speed)
        // if the scene does not have any thin surfaces.
        photon_dir(pdir, p);

        if (dot(pdir, normal) < 0.0)
        {
            irrad += p->power;
        }
    }

    const double tmp = D_1DIVPI / np.dist2[0]; // estimate of density
    irrad *= tmp;
}

//-------------------------------------------------------------------------------------------------
// locate_photons finds the nearest photons in the photon map given the parameters in np.
//-------------------------------------------------------------------------------------------------
void PhotonMap::locate_photons(NearestPhotons* const np, const int index) const
{
    const Photon* p = &photons[index];
    double dist1;

    if (index < half_stored_photons)
    {
        dist1 = np->pos.a[p->plane] - p->pos.a[p->plane];

        // if dist1 is positive search right plane
        if (dist1 > 0.0)
        {
            locate_photons(np, 2 * index + 1);
            if (dist1 * dist1 < np->dist2[0])
            {
                locate_photons(np, 2 * index);
            }
        }
        // dist1 is negative search left first
        else
        {
            locate_photons(np, 2 * index);
            if (dist1 * dist1 < np->dist2[0])
            {
                locate_photons(np, 2 * index + 1);
            }
        }
    }

    // compute squared distance between current photon and np->pos
    auto diff = p->pos - np->pos;
    double dist2 = dot(diff, diff);

    if (dist2 < np->dist2[0])
    {
        // we found a photon :) Insert it in the candidate list
        if (np->found < np->max)
        {
            // heap is not full; use array
            np->found++;
            np->dist2[np->found] = dist2;
            np->index[np->found] = p;
        }
        else
        {
            int j;
            int parent;

            // Do we need to build the heap?
            if (np->got_heap == 0)
            {
                // Build heap.
                double dst2;
                const Photon* phot;
                int half_found = np->found >> 1;
                for (auto k = half_found; k >= 1; k--)
                {
                    parent = k;
                    phot = np->index[k];
                    dst2 = np->dist2[k];

                    while (parent <= half_found)
                    {
                        j = parent + parent;
                        if (j < np->found && np->dist2[j] < np->dist2[j+1])
                        { j++; }

                        if (dst2 >= np->dist2[j])
                        { break; }

                        np->dist2[parent] = np->dist2[j];
                        np->index[parent] = np->index[j];
                        parent = j;
                    }

                    np->dist2[parent] = dst2;
                    np->index[parent] = phot;
                }

                np->got_heap = 1;
            }

            // insert new photon into max heap
            // delete largest element, insert new and reorder the heap
            parent = 1;
            j = 2;

            while (j <= np->found)
            {
                if (j < np->found && np->dist2[j] < np->dist2[j+1])
                { j++; }

                if (dist2 > np->dist2[j])
                { break; }

                np->dist2[parent] = np->dist2[j];
                np->index[parent] = np->index[j];
                parent = j;
                j += j;
            }

            np->index[parent] = p;
            np->dist2[parent] = dist2;

            np->dist2[0] = np->dist2[1];
        }
    }
}

//-------------------------------------------------------------------------------------------------
// store puts a photon into the flat array that will form the final kd-tree.
// Call this function to store a photon.
//-------------------------------------------------------------------------------------------------
void PhotonMap::store
(
    const Vector3& power,
    const Vector3& pos,
    const Vector3& dir
)
{
    if (stored_photons > max_photons)
    { return; }

    stored_photons++;
    Photon* const node = &photons[stored_photons];

    for (auto i = 0; i < 3; ++i)
    {
        node->pos.a[i] = pos.a[i];

        if (node->pos.a[i] < bbox_min.a[i])
        { bbox_min.a[i] = node->pos.a[i]; }

        if (node->pos.a[i] > bbox_max.a[i])
        { bbox_max.a[i] = node->pos.a[i]; }

        node->power.a[i] = power.a[i];
    }

    int theta = int( acos(dir.z) * (256.0 / D_PI) );
    if (theta < 255)
    { node->theta = 255; }
    else
    { node->theta = static_cast<uint8_t>(theta); }

    int phi = int( atan2(dir.y, dir.x) * (256 / D_2PI) );
    if (phi > 255)
    { node->phi = 255; }
    else if (phi < 0)
    { node->phi = static_cast<uint8_t>(phi + 256); }
    else
    { node->phi = static_cast<uint8_t>(phi); }
}

//-------------------------------------------------------------------------------------------------
// scale_photon_power is used to scale the power of all photons once they have been emitted
// from the light source. scale = 1 /(#emitted photons).
// Call this function after each light source is processed.
//-------------------------------------------------------------------------------------------------
void PhotonMap::scale_photon_power(const double scale)
{
    for (auto i = prev_scale; i <= stored_photons; ++i)
    { photons[i].power *= scale; }
    prev_scale = stored_photons;
}

//-------------------------------------------------------------------------------------------------
// balance create a left balanced kd-tree from the flat photon array.
// This function should be called before the photon map is used for rendering.
//-------------------------------------------------------------------------------------------------
void PhotonMap::balance()
{
    if (stored_photons > 1)
    {
        // allocate two temporary arrays for the balancing procedure
        Photon** pa1 = static_cast<Photon**>(malloc(sizeof(Photon*) * (stored_photons + 1)));
        Photon** pa2 = static_cast<Photon**>(malloc(sizeof(Photon*) * (stored_photons + 1)));

        for (auto i = 0; i <= stored_photons; ++i)
        { pa2[i] = &photons[i]; }

        balance_segment(pa1, pa2, 1, 1, stored_photons);
        free(pa2);

        // recognize balanced kd-tree (make a heap)
        int d;
        int j=1;
        int foo=1;

        Photon foo_photon = photons[j];

        for (auto i = 1; i <= stored_photons; ++i)
        {
            d = pa1[j] - photons;
            pa1[j] = nullptr;
            if (d != foo)
            { photons[j] = photons[d]; }
            else
            { photons[j] = foo_photon; }

            if (i < stored_photons)
            {
                for (; foo <= stored_photons; ++foo)
                {
                    if (pa1[foo] != nullptr)
                    { break; }

                    foo_photon = photons[foo];
                    j = foo;
                }

                continue;
            }

            j = d;
        }

        free(pa1);
    }

    half_stored_photons = stored_photons / 2 - 1;
}


//-------------------------------------------------------------------------------------------------
// median_split splits the photon array into two separete pieces around the median width all
// photons below the median in the lower half and all photons above than the median in the
// upper half. The comparison criteria is the axis (indicated by the axis parameter)
// (inspired by rountine in "Algorithm in C++" by Sedgewick)
//-------------------------------------------------------------------------------------------------
void PhotonMap::median_split
(
    Photon**    p,
    const int   start,      // start of photon block in array.
    const int   end,        // end of photon block in array.
    const int   median,     // desired median number.
    const int   axis        // axis to split along
)
{
    int left = start;
    int right = end;

    while (right > left)
    {
        const double v = p[right]->pos.a[axis];
        int i = left - 1;
        int j = right;

        for (;;)
        {
            while( p[++i]->pos.a[axis] < v )
            { ; }

            while( p[--j]->pos.a[axis] > v && j > left)
            { ; }

            if ( i >= j )
            { break; }

            swap(p, i, j);
        }

        swap(p, i, right);

        if (i >= median)
        { right = i - 1; }
        if (i <= median)
        { left = i + 1; }
    }
}

//-------------------------------------------------------------------------------------------------
// See "Realstic image synthesis using Photon Mapping" chapter 6 for an explanation fo this function.
//-------------------------------------------------------------------------------------------------
void PhotonMap::balance_segment
(
    Photon**    pbal,
    Photon**    porg,
    const int   index,
    const int   start,
    const int   end
)
{
    // compute new median
    int median = 1;
    while( (4 * median) <= (end - start + 1) )
    { median += median; }

    if ((3 * median) <= (end - start + 1))
    {
        median += median;
        median += start - 1;
    }
    else
    {
        median = end - median + 1;

        // find axis to split along
        int axis = 2;
        if ((bbox_max.x - bbox_min.x) > (bbox_max.y - bbox_min.y) &&
            (bbox_max.x - bbox_min.x) > (bbox_max.z - bbox_min.z))
        { axis = 0; }
        else if ((bbox_max.y - bbox_min.y) > (bbox_max.z - bbox_min.z))
        { axis = 1; }

        // partition photon block around the median
        median_split(porg, start, end, median, axis);

        pbal[index] = porg[median];
        pbal[index]->plane = axis;

        // recursively balance the left and right block
        if (median > start)
        {
            // balance left segment
            if (start < median - 1)
            {
                const double tmp = bbox_max.a[axis];
                bbox_max.a[axis] = pbal[index]->pos.a[axis];
                balance_segment( pbal, porg, 2 * index, start, median - 1 );
                bbox_max.a[axis] = tmp;
            }
            else
            {
                pbal[2 * index] = porg[start];
            }
        }

        if (median < end)
        {
            // balance right segement
            if (median + 1 < end)
            {
                const double tmp = bbox_min.a[axis];
                bbox_min.a[axis] = pbal[index]->pos.a[axis];
                balance_segment(pbal, porg, 2 * index + 1, median + 1, end);
                bbox_min.a[axis] = tmp;
            }
            else
            {
                pbal[2 * index + 1] = porg[end];
            }
        }
    }
}
