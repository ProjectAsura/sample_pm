//-------------------------------------------------------------------------------------------------
// File : photon_map.h
// Desc : Photon Map Module.
// Original Code by hole (https://github.com/githole/simple-photonmap)
//-------------------------------------------------------------------------------------------------
#pragma once

#include <r3d_math.h>
#include <r3d_stack_allocator.h>
#include <queue>
#include <vector>


struct photon
{
    Vector3 pos;
    Vector3 flux;
    Vector3 dir;
};

struct photon_query
{
    double      max_dist2;
    size_t      count;
    Vector3     pos;
    Vector3     normal;
};

struct photon_query_result
{
    const photon*   point;
    double          dist2;

    photon_query_result(const photon* p, double d2)
    : point (p)
    , dist2 (d2)
    { /* DO_NOTHING */ }

    bool operator < (const photon_query_result& value) const
    { return dist2 < value.dist2; }
};

class nearest_photon
{
public:
    nearest_photon(size_t size)
    : m_photons()
    , m_less()
    { m_photons.reserve(size + 1); }

    ~nearest_photon()
    {
        m_photons.clear();
        m_photons.shrink_to_fit();
    }

    void push(const photon_query_result& value)
    {
        m_photons.emplace_back(value);
        std::push_heap(m_photons.begin(), m_photons.end(), m_less);
    }

    bool empty() const
    { return m_photons.empty(); }

    size_t size() const
    { return m_photons.size(); }

    const photon_query_result& top() const
    { return m_photons.front(); }

    void pop()
    {
        std::pop_heap(m_photons.begin(), m_photons.end(), m_less);
        m_photons.pop_back();
    }

    const photon_query_result& operator[](int index)
    { return m_photons[index]; }

private:
    std::vector<photon_query_result>    m_photons;
    std::less<photon_query_result>      m_less;
};


class photon_map
{
public:
    photon_map()
    : m_root(nullptr)
    { /* DO_NOTHING */ }

    ~photon_map()
    { clear(); }

    size_t size() const
    { return m_points.size(); }

    void search(photon_query& query, nearest_photon& result)
    { recursive_search(query, m_root, result); }

    void store(const photon& value)
    { m_points.emplace_back(value); }

    void build()
    {
        m_points.shrink_to_fit();
        m_root = recursive_build(m_points.data(), int(m_points.size()), 0);
    }

    void clear()
    { recursive_clear(m_root); }

private:
    struct node
    {
        photon* point;
        node*   left;
        node*   right;
        int     axis;
    };

    struct less_comp
    {
        less_comp(int axis)
        : axis_(axis)
        { /* DO_NOTHING */ }

        bool operator() (const photon& lhs, const photon& rhs)
        { return lhs.pos.a[axis_] < rhs.pos.a[axis_]; }
 
        int axis_;
    };

    node*                   m_root;
    std::vector<photon>     m_points;

    void recursive_search(photon_query& query, node* node, nearest_photon& result)
    {
        if (node == nullptr)
        { return; }

        const int  axis  = node->axis;
        double     delta = query.pos.a[axis] - node->point->pos.a[axis];
        const auto dir   = node->point->pos - query.pos;
        const auto dist2 = dot(dir, dir);
        const auto dt    = dot(query.normal, dir / sqrt(dist2));

        if (dist2 < query.max_dist2 && fabs(dt) <= query.max_dist2 * 0.01)
        {
            result.push(photon_query_result(node->point, dist2));
            if (result.size() > query.count)
            {
                result.pop();
                query.max_dist2 = result.top().dist2;
            }
        }

        if (delta > 0.0)
        {
            recursive_search(query, node->right, result);
            if (delta * delta < query.max_dist2)
            { recursive_search(query, node->left, result); }
        }
        else
        {
            recursive_search(query, node->left, result);
            if (delta * delta < query.max_dist2)
            { recursive_search(query, node->right, result); }
        }
    }

    node* recursive_build(photon* photons, int count, int depth)
    {
        if (count <= 0)
        { return nullptr; }

        const int axis = depth % 3;
        const int mid = (count - 1) / 2;

        std::nth_element(photons, photons + mid, photons + count, less_comp(axis));

        node* node  = new(std::nothrow) photon_map::node();
        node->axis  = axis;
        node->point = &photons[mid];
        node->left  = recursive_build(photons, mid, depth + 1);
        node->right = recursive_build(photons + mid + 1, count - mid - 1, depth + 1);

        return node;
    }

    void recursive_clear(node* node)
    {
        if (node == nullptr)
        { return; }

        recursive_clear(node->left);
        recursive_clear(node->right);
        delete node;
    }
};