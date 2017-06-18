//------------------------------------------------------------------------------------------------
// File : photonmap.cpp
// Desc : Photon Map Data structure.
// Copyright(c) Project Asura. All right reserved.
//-------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------
// Includes
//-------------------------------------------------------------------------------------------------
#include "photonmap.h"


namespace {

inline bool comp_x(const Photon& lhs, const Photon& rhs)
{ return lhs.pos.x < rhs.pos.x; }

inline bool comp_y(const Photon& lhs, const Photon& rhs)
{ return lhs.pos.y < rhs.pos.y; }

inline bool comp_z(const Photon& lhs, const Photon& rhs)
{ return lhs.pos.z < rhs.pos.z; }

} // namespace

///////////////////////////////////////////////////////////////////////////////////////////////////
// PhotonMap class
///////////////////////////////////////////////////////////////////////////////////////////////////

//-------------------------------------------------------------------------------------------------
//      コンストラクタです.
//-------------------------------------------------------------------------------------------------
PhotonMap::PhotonMap()
: m_root(nullptr)
{ /* DO_NOTHING */ }

//-------------------------------------------------------------------------------------------------
//      デストラクタです.
//-------------------------------------------------------------------------------------------------
PhotonMap::~PhotonMap()
{ clear(); }

//-------------------------------------------------------------------------------------------------
//      kd-treeを構築します.
//-------------------------------------------------------------------------------------------------
void PhotonMap::build()
{ m_root = recusrive_build(m_photons.begin(), m_photons.end(), 0); }

//-------------------------------------------------------------------------------------------------
//      破棄処理.
//-------------------------------------------------------------------------------------------------
void PhotonMap::clear()
{
    recursive_clear(m_root);
    m_photons.clear();
    m_photons.shrink_to_fit();
}

//-------------------------------------------------------------------------------------------------
//      フォトン数を取得します.
//-------------------------------------------------------------------------------------------------
size_t PhotonMap::size() const
{ return m_photons.size(); }

//-------------------------------------------------------------------------------------------------
//      フォトンマップに格納します.
//-------------------------------------------------------------------------------------------------
void PhotonMap::store(const Vector3& pos, const Vector3& flux)
{
    Photon value = {};
    value.pos  = pos;
    value.flux = flux;
    m_photons.push_back(value);
}

//-------------------------------------------------------------------------------------------------
//      最近傍フォトンを K-nearest neighbor により求めます.
//-------------------------------------------------------------------------------------------------
void PhotonMap::search(const Vector3& pos, double max_dist, size_t count, QueryQueue& query)
{ recursive_search(pos, max_dist, count, m_root, query); }

//-------------------------------------------------------------------------------------------------
//      Median-Split による再帰ビルド.
//-------------------------------------------------------------------------------------------------
PhotonMap::Node* PhotonMap::recusrive_build
(
    PhotonArray::iterator   begin,
    PhotonArray::iterator   end,
    int                     depth
)
{
    if (end - begin <= 0)
    { return nullptr; }

    const auto axis = depth % 3;

    switch (axis)
    {
    case 0 : { std::sort(begin, end, comp_x); } break;
    case 1 : { std::sort(begin, end, comp_y); } break;
    case 2 : { std::sort(begin, end, comp_z); } break;
    }

    const auto mid = (end - begin) / 2;

    auto node = new Node();
    node->axis = axis;
    node->value = &(*(begin + mid));
    node->left  = recusrive_build( begin, begin + mid, depth + 1 );
    node->right = recusrive_build( begin + mid + 1, end, depth + 1 );
    return node;
}

//-------------------------------------------------------------------------------------------------
//      再帰クリア
//-------------------------------------------------------------------------------------------------
void PhotonMap::recursive_clear(Node* node)
{
    if (node == nullptr)
    { return; }

    if (node->left != nullptr)
    { recursive_clear(node->left); }

    if (node->right != nullptr)
    { recursive_clear(node->right); }

    delete node;
}

//-------------------------------------------------------------------------------------------------
//      再帰検索.
//-------------------------------------------------------------------------------------------------
void PhotonMap::recursive_search
(
    const Vector3&          pos,
    double&                 max_dist,
    size_t                  count,
    Node*                   node,
    PhotonMap::QueryQueue&  query
)
{
    if (node == nullptr)
    { return; }

    const auto axis = node->axis;
    double delta;

    switch (axis)
    {
        case 0: { delta = pos.x - node->value->pos.x; } break;
        case 1: { delta = pos.y - node->value->pos.y; } break;
        case 2: { delta = pos.z - node->value->pos.z; } break;
    }

    const auto dir   = node->value->pos - pos;
    const auto dist2 = dot(dir, dir);

    if (dist2 < max_dist)
    {
        QueryResult result = {};
        result.photon = node->value;
        result.dist   = dist2;

        query.push(result);

        if (query.size() > count)
        {
            query.pop();
            max_dist = query.top().dist;
        }
    }
    if (delta > 0.0)
    {
        recursive_search(pos, max_dist, count, node->right, query);
        if (delta * delta < max_dist) { recursive_search(pos, max_dist, count, node->left, query); }
    }
    else
    {
        recursive_search(pos, max_dist, count, node->left, query);
        if (delta * delta < max_dist){ recursive_search(pos, max_dist, count, node->right, query); }
    }
}
