#ifndef ASTAR_HPP
#define ASTAR_HPP

#include <vector>
#include <set>
#include <stack>
#include "assert.h"
#include <algorithm>
#include <Eigen/Core>

namespace AStar
{
// A struct to represent two dimensional coordinates.
    struct Vec2d
    {
        //Vec2d(int x =-1, int y = -1) : x(x), y(y) {}
        int x, y;

        bool operator==(const Vec2d &coordinate_);

        Vec2d operator+(const Vec2d &coordinate_);

        Vec2d operator-(const Vec2d &coordinate_);
    };

    struct pointCoord
    {
        pointCoord(double x = -1.0, double y = -1.0) : x(x), y(y)
        {}

        pointCoord(Vec2d p);

        bool operator==(const pointCoord &coordinate_);

        pointCoord operator+(const pointCoord &a);

        pointCoord operator-(const pointCoord &a);

        double operator*(const pointCoord &a);

        pointCoord operator*(const double &b);

        double x, y;
    };

// A collection of 2d coordinates represented by a vector.
    using CoordinateList = std::vector<Vec2d>;

/* A struct to hold the necessary parameters of nodes.
   {
     coordinate: the location of the node in the map.
     G: the movement cost to move from the starting node to current node, following the path generated to get there.
     H: the estimated movement cost to move from current node to the final destination.
     F: the total cost, F = G + H.
     parent: the parent node of current node.
   }
*/
    struct Node
    {
        Vec2d coordinate;
        double G, H, F;
        Node *parent;

        Node(Vec2d coordinate_, Node *parent_ = nullptr);
    };

// A struct to indicate the comparative rule for sorting Nodes in a std::set<Node*>.
// Default arrangement: small to large.
    struct NodeCmp
    {
        bool operator()(const Node *a, const Node *b) const
        {
            if (a->F != b->F)
            {
                return a->F < b->F;
            }
                /*		else if (a->G != b->G)
                {
                    return a->G < b->G;
                }
        */
            else if (a->coordinate.x != b->coordinate.x)
            {
                return a->coordinate.x < b->coordinate.x;
            }
            else if (a->coordinate.y != b->coordinate.y)
            {
                return a->coordinate.y < b->coordinate.y;
            }
            else
            {
                return false;
            }
        }
    };

// std::set<>, representing a collection of Nodes arranged according to the value of F.
    using NodeSet = std::set<Node *, NodeCmp>;

// vector<vector<int>>, representing a 2d array.
    using VecVec = std::vector<std::vector<int>>;

    using VecVecIP = std::vector<std::vector<Node *>>;

    class baseMap
    {
    public:
        baseMap();

        baseMap(const VecVec &GridState_, int robot_block_radius_ = 0) : robot_block_radius(robot_block_radius_)
        { setGridMap(GridState_); }

        ~baseMap()
        {}

//	static VecVec AddBorderLineMap(const VecVec &GridState_, double x0, double y0, double delta_x, double delta_y, bool border_direction);
//    static void AddBorderLineMap(VecVec &GridState_, double x0, double y0, double delta_x, double delta_y, bool border_direction);
        VecVec DistanceMap(const VecVec &GridState_);

        void setGridMap(const VecVec &GridState_);

        VecVec getGridMap();

        bool inMap(const Vec2d point_);

        bool findNearFreePoint(Vec2d &point, int extend_count = -1);

        bool reprocessTarget(const Vec2d &start_, Vec2d &target_);
//    Vec2d reprocessTarget(int targetX_, int targetY);

        void setRobotSelfRadius(int robot_block_radius_);

        bool detectCollision(const Vec2d &coordinate_);

        bool reachCorner(Vec2d curr_, Vec2d succ_);

        bool crossPointCollision(pointCoord p_);

        bool lineCollision(pointCoord p1_, pointCoord p2_);

    protected:
        Vec2d gridSize;
        VecVec GridState;
        int robot_block_radius;
        std::vector<Eigen::Vector2i> border_free_points;
    };

/**/
    class Planning : virtual public baseMap
    {
//	bool reachCorner(Vec2d curr_, Vec2d succ_);
        double calHeurCost(Vec2d startpoint_, Vec2d endpoint_);

        void releaseNodes(NodeSet &nodes_);
//	bool detectCollision(Vec2d coordinate_);
//	bool lineCollision(pointCoord p1_, pointCoord p2_);

    public:
        Planning();

        Planning(const VecVec &GridState_, int robot_block_radius_ = 0) : baseMap(GridState_, robot_block_radius_)
        {}

        ~Planning()
        {}

        void setGridMap(const VecVec &GridState_);

//	static VecVec DistanceMap(VecVec GridState_);
//	bool crossPointCollision(pointCoord p_);
//	VecVec getGridMap();
        CoordinateList
        findPath(Vec2d source, Vec2d target, Vec2d init_direction = {1, 0}, int init_len = 3, double distance_threshold = 0.0, bool raw = false);

        CoordinateList simplePath(Vec2d source_, Vec2d target_, Vec2d init_direction = {1, 0}, int init_len = 3, double distance_threshold = 0.0);

        void showPath(CoordinateList path_);

        struct astar_cost
        {
            float weight_g = 1.2;
            float weight_f = 0.8;
            float turn = 1.5;
            float move_diag = 1.414;
            float weight_distant = 20.0;
            int max_distant_grid = 10;
        };

        void setAstarCost(float weight_g_, float weight_f_, float turn_, float move_diag_, float weight_distant_, int max_distant_grid_);

        void setAstarCost(astar_cost astar_cost_factors_);

    protected:
//	Vec2d gridSize;
//	VecVec GridState;
        CoordinateList moveDirections;
        astar_cost astar_cost_factors;
    };

    class BSpline
    {
        double NU(int i_, int k_, double t);

    public:
        BSpline(CoordinateList path_, int k_);

        ~BSpline()
        {}

        pointCoord PU(double t_);

    private:
        CoordinateList path;
        int n, k;
        std::vector<double> u;
    };

    struct arcInfo
    {
        int rotFlag;
        double lenR;
        double theta;
        pointCoord startCoord;
        pointCoord endCoord;
        pointCoord midCoord;
        pointCoord centreCoord;
    };

    using VecVecD = std::vector<std::vector<double>>;

    class Arc : virtual public baseMap
    {
        arcInfo singleArc(pointCoord startCoord_, pointCoord midCoord_, pointCoord endCoord_, double Rd = 0,
                          bool detectC = true);

        arcInfo Line(pointCoord startCoord_, pointCoord endCoord_);

//	std::vector<arcInfo> doubleArc(pointCoord startCoord_, pointCoord startDirec_, pointCoord endCoord_, pointCoord endDirec_);
        bool point_in_arc(pointCoord p_, arcInfo arc_);

        bool arcCollision(arcInfo arc_);


    public:
        Arc() : baseMap()
        {}

        Arc(const VecVec &GridState_, int robot_block_radius_ = 0) : baseMap(GridState_, robot_block_radius_)
        {}

        ~ Arc()
        {}

        void setGridMap(const VecVec &GridState_);

        static VecVecD info2vec(const std::vector<arcInfo> &arc_);

        std::vector<arcInfo> ArcSmooth(CoordinateList path_, double Rd = 0);
    };

    class ArcPlanning : public Arc, public Planning
    {
    public:
        ArcPlanning() : baseMap(), Arc(), Planning()
        {}

        ArcPlanning(const VecVec &GridState_, int robot_block_radius_ = 0) : baseMap(), Arc(), Planning()
        {
            setGridMap(GridState_);
            setRobotSelfRadius(robot_block_radius_);
        }

        ~ArcPlanning()
        {}

        void setGridMap(const VecVec &GridState_);

        std::vector<arcInfo>
        arc_planning(int sourceX, int sourceY, int targetX, int targetY, int init_direcX = 0, int init_direcY = 1,
                     int init_len = 10, double Rd = 0.0, double distance_threshold = 0.0);
    };

    std::vector<std::vector<float>> path2cmd(const std::vector<arcInfo> &path_, float resolution, float vel, float w);

    std::vector<std::vector<float>>
    getPathCommand(const VecVec &gridVector, int sourceX, int sourceY, int targetX, int targetY,
                   int robot_block_radius_ = 0, int init_direcX = 0, int init_direcY = 1, int init_len = 10,
                   double Rd = 40.0, float resolution = 0.1, float vel = 0.5, float w = 0.3);

    VecVecD path_planning(VecVec gridVector, int sourceX, int sourceY, int targetX, int targetY, int init_direcX = 0,
                          int init_direcY = 1, int init_len = 10, double Rd = 0.0);

    void print_path(VecVecD path);

    std::vector<std::vector<int>>
    raw_path(VecVec gridVector, int sourceX, int sourceY, int targetX, int targetY, int init_direcX = 0,
             int init_direcY = 1, int init_len = 10, bool raw = false);

//生成显示路径曲线用的点
    std::vector<pointCoord> path2points(std::vector<arcInfo> path_, float deltaLen);

    std::vector<pointCoord>
    getPathPoints(VecVec gridVector, int sourceX, int sourceY, int targetX, int targetY, int init_direcX = 0,
                  int init_direcY = 1, int init_len = 10, double Rd = 40.0, float deltaLen = 0.1);

} // namespace AStar
#endif
