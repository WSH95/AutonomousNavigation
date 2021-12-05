#include "AStar.hpp"
#include <math.h>
#include <algorithm>
#include <iostream>
#include <iomanip>

using namespace std;
using namespace AStar;

bool Vec2d::operator==(const Vec2d &coordinate_)
{
    return (x == coordinate_.x && y == coordinate_.y);
}

Vec2d Vec2d::operator+(const Vec2d &coordinate_)
{
    return {x + coordinate_.x, y + coordinate_.y};
}

Vec2d Vec2d::operator-(const Vec2d &coordinate_)
{
    return {x - coordinate_.x, y - coordinate_.y};
}

pointCoord operator*(const Vec2d &a, const double &b)
{
    return {a.x * b, a.y * b};
}

Node::Node(Vec2d coordinate_, Node *parent_)
{
    parent = parent_;
    coordinate = coordinate_;
    G = H = F = 0.0;
}

baseMap::baseMap()
{
    gridSize = {1, 1};
    GridState = {{0}};
    robot_block_radius = 0; //planning_cfg.get_self_block_radius_grid();
}
//
//VecVec baseMap::AddBorderLineMap(const VecVec &GridState_, double x0, double y0, double delta_x, double delta_y, bool border_direction)
//{
//    VecVec tmpMap(GridState_);
//
//    // TODO update the grid map.
//
//
//    return tmpMap;
//}

/**/
VecVec baseMap::DistanceMap(const VecVec &GridState_)
{
    VecVec tmpMap = {{0}};
    tmpMap.resize(int(GridState_.size()));
    for (auto &elem: tmpMap)
    {
        elem.resize(int(GridState_[0].size()), 0);
    }

    CoordinateList tmpCoord;
    int tmpDistance = 1;
    CoordinateList tmpDirections = {
            {0,  1},
            {1,  0},
            {0,  -1},
            {-1, 0},
            {-1, -1},
            {1,  1},
            {-1, 1},
            {1,  -1}};

    for (int ii = 0; ii < tmpMap.size(); ++ii)
    {
        for (int jj = 0; jj < tmpMap[0].size(); ++jj)
        {
            if (GridState_[ii][jj] == 1)
            {
                tmpMap[ii][jj] = -1;
            }
            else if (GridState_[ii][jj] == 2)
            {
                tmpMap[ii][jj] = -2;
            }
            else
            {
                tmpCoord.push_back({ii, jj});
            }
        }
    }
    if (tmpCoord.size() > 0)
    {
        for (auto it = tmpCoord.begin(); it != tmpCoord.end();)
        {
            bool flag = false;
            for (int iii = 0; iii < 8; ++iii)
            {
                int XX = it->x;
                int YY = it->y;
                int newXX = XX + tmpDirections[iii].x;
                int newYY = YY + tmpDirections[iii].y;

                // push the available points on border to a vector.
                if (newXX < 0 || newXX >= tmpMap.size() ||
                    newYY < 0 || newYY >= tmpMap[0].size())
                {
                    if (robot_block_radius < tmpDistance)
                    {
                        Eigen::Vector2i a(XX, YY);
                        border_free_points.push_back(a);
                    }
                    continue;
                }
                if (tmpMap[newXX][newYY] < 0)
                {
                    tmpMap[XX][YY] = 1;
                    it = tmpCoord.erase(it);
                    flag = true;
                    break;
                }
                else
                {
                    continue;
                }
            }
            if (!flag)
            {
                ++it;
            }
        }
        ++tmpDistance;
    }
    while (tmpCoord.size() > 0)
    {
        for (auto it = tmpCoord.begin(); it != tmpCoord.end();)
        {
            bool flag = false;
            for (int iii = 0; iii < 8; ++iii)
            {
                int XX = it->x;
                int YY = it->y;
                int newXX = XX + tmpDirections[iii].x;
                int newYY = YY + tmpDirections[iii].y;

                // push the available points on border to a vector.
                if (newXX < 0 || newXX >= tmpMap.size() ||
                    newYY < 0 || newYY >= tmpMap[0].size())
                {
                    if (robot_block_radius < tmpDistance)
                    {
                        Eigen::Vector2i a(XX, YY);
                        border_free_points.push_back(a);
                    }
                    continue;
                }

                if (tmpMap[newXX][newYY] == tmpDistance - 1)
                {
                    tmpMap[XX][YY] = tmpDistance;
                    it = tmpCoord.erase(it);
                    flag = true;
                    break;
                }
                else
                {
                    continue;
                }
            }
            if (!flag)
            {
                ++it;
            }
        }
        ++tmpDistance;
    }

    return tmpMap;
}

void baseMap::setGridMap(const VecVec &GridState_)
{
    gridSize = {int(GridState_.size()), int(GridState_[0].size())};
    GridState = DistanceMap(GridState_);

    /*	GridState.resize(int(GridState_.size()));
    //copy(GridState_.begin(), GridState_.end(), GridState.begin());
    //////////////////////////////////////////////////////////////////////////////////////////
    CoordinateList tmpCoord;
    int tmpDistance = 1;
    for (auto &elem : GridState)
    {
        elem.resize(int(GridState_[0].size()), 0);
    }
    for (int ii = 0; ii < GridState.size(); ++ii)
    {
        for (int jj = 0; jj < GridState[0].size(); ++jj)
        {
            if (GridState_[ii][jj] > 0)
            {
                GridState[ii][jj] = -1;
            }
            else
            {
                tmpCoord.push_back({ii, jj});
            }
        }
    }
    if (tmpCoord.size() > 0)
    {
        for (auto it = tmpCoord.begin(); it != tmpCoord.end();)
        {
            bool flag = false;
            for (int iii = 0; iii < 8; ++iii)
            {
                int XX = it->x;
                int YY = it->y;
                Vec2d newCoord(Vec2d{XX, YY} + moveDirections[iii]);
                if (detectCollision(newCoord))
                {
                    GridState[XX][YY] = tmpDistance;
                    it = tmpCoord.erase(it);
                    flag = true;
                    break;
                }
                else
                {
                    continue;
                }
            }
            if (!flag)
            {
                ++it;
            }
        }
        ++tmpDistance;
    }
    while (tmpCoord.size() > 0)
    {
        for (auto it = tmpCoord.begin(); it != tmpCoord.end();)
        {
            bool flag = false;
            for (int iii = 0; iii < 8; ++iii)
            {
                int XX = it->x;
                int YY = it->y;
                Vec2d newCoord(Vec2d{XX, YY} + moveDirections[iii]);
                if (GridState[newCoord.x][newCoord.y] == tmpDistance - 1)
                {
                    GridState[XX][YY] = tmpDistance;
                    it = tmpCoord.erase(it);
                    flag = true;
                    break;
                }
                else
                {
                    continue;
                }
            }
            if (!flag)
            {
                ++it;
            }
        }
        ++tmpDistance;
    }
    //////////////////////////////////////////////////////////////////////////////////////////
*/
}

void baseMap::setRobotSelfRadius(int robot_block_radius_)
{
    robot_block_radius = robot_block_radius_;
}

VecVec baseMap::getGridMap()
{
    return GridState;
}

bool baseMap::detectCollision(const Vec2d &coordinate_)
{
    if ((coordinate_.x < 0) || (coordinate_.x >= gridSize.x) ||
        (coordinate_.y < 0) || (coordinate_.y >= gridSize.y) ||
        (GridState[coordinate_.x][coordinate_.y] <= robot_block_radius))
    {
//        std::cout << "block" << endl;
        return true;
    }
    else
    {
//        std::cout << "free" << endl;
        return false;
    }
}

bool baseMap::reachCorner(Vec2d curr_, Vec2d succ_)
{
    if (GridState[curr_.x][succ_.y] < 0 && GridState[succ_.x][curr_.y] < 0)
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool baseMap::crossPointCollision(pointCoord p_)
{
    CoordinateList pp;
    if (abs(p_.x - int(p_.x)) == 0.5 && abs(p_.y - int(p_.y)) == 0.5)
    {
        pp.push_back({int(p_.x - 0.5), int(p_.y - 0.5)});
        pp.push_back({int(p_.x - 0.5), int(p_.y + 0.5)});
        pp.push_back({int(p_.x + 0.5), int(p_.y - 0.5)});
        pp.push_back({int(p_.x + 0.5), int(p_.y + 0.5)});
    }
    else if (abs(p_.x - int(p_.x)) == 0.5)
    {
        pp.push_back({int(p_.x - 0.5), int(p_.y + 0.5)});
        pp.push_back({int(p_.x + 0.5), int(p_.y + 0.5)});
    }
    else if (abs(p_.y - int(p_.y)) == 0.5)
    {
        pp.push_back({int(p_.x + 0.5), int(p_.y - 0.5)});
        pp.push_back({int(p_.x + 0.5), int(p_.y + 0.5)});
    }
    else
    {
        cout << "Error in coordinate of the intersection point!" << endl;
        exit(0);
    }

    for (int i = 0; i < pp.size(); ++i)
    {
        if (detectCollision(pp[i]))
        {
            return true;
        }
    }

    return false;
}

bool baseMap::lineCollision(pointCoord p1_, pointCoord p2_)
{
    std::vector<pointCoord> ppp;
    pointCoord start_;
    pointCoord end_;
    Vec2d pp1 = {int(p1_.x + 0.5), int(p1_.y + 0.5)};
    Vec2d pp2 = {int(p2_.x + 0.5), int(p2_.y + 0.5)};
    if (detectCollision(pp1) || detectCollision(pp2))
    {
        return true;
    }
    if (pp1 == pp2)
    {
        return detectCollision(pp1);
    }
    else if (pp1.x == pp2.x)
    {
        if (p1_.y < p2_.y)
        {
            start_ = p1_;
            end_ = p2_;
        }
        else
        {
            start_ = p2_;
            end_ = p1_;
        }
        for (int ii = int(start_.y + 0.5); ii <= int(end_.y + 0.5); ++ii)
        {
            if (detectCollision({pp1.x, ii}))
            {
                return true;
            }
            else
            {
                continue;
            }
        }
    }
    else if (pp1.y == pp2.y)
    {
        if (p1_.x < p2_.x)
        {
            start_ = p1_;
            end_ = p2_;
        }
        else
        {
            start_ = p2_;
            end_ = p1_;
        }
        for (int ii = int(start_.x + 0.5); ii <= int(end_.x + 0.5); ++ii)
        {
            if (detectCollision({ii, pp1.y}))
            {
                return true;
            }
            else
            {
                continue;
            }
        }
    }
    else
    {
        double k = (p2_.y - p1_.y) / (p2_.x - p1_.x);

        if (p1_.x < p2_.x)
        {
            start_ = p1_;
            end_ = p2_;
        }
        else
        {
            start_ = p2_;
            end_ = p1_;
        }
        if (abs(k) <= 1)
        {
            int n = abs(pp1.x - pp2.x);
            for (int i = 0; i < n; ++i)
            {
                double xx = int(start_.x + 0.5) + 0.5 + i;
                double yy = k * (xx - start_.x) + start_.y;
                pointCoord tmp_(xx, yy);
                if (crossPointCollision(tmp_))
                {
                    return true;
                }
                else
                {
                    continue;
                }

            }
        }
        else
        {
            int n = abs(pp1.y - pp2.y);
            int flag = k > 0 ? 1 : -1;
            for (int i = 0; i < n; ++i)
            {
                double yy = int(start_.y + 0.5) + (0.5 + i) * flag;
                double xx = (1 / k) * (yy - start_.y) + start_.x;
                pointCoord tmp_(xx, yy);
                if (crossPointCollision(tmp_))
                {
                    return true;
                }
                else
                {
                    continue;
                }

            }
        }
    }

    return false;
}

bool baseMap::inMap(const Vec2d point_)
{
    if ((point_.x < 0) || (point_.x >= gridSize.x) || (point_.y < 0) || (point_.y >= gridSize.y))
        return false;
    else
        return true;
}

bool baseMap::findNearFreePoint(Vec2d &point, int extend_count)
{
    if (!detectCollision(point))
    {
//        std::cout << "free" << std::endl;
        return true;
    }

//    std::cout << "Collision" << std::endl;

    if (!inMap(point))
    {
        std::cout << "[Error: findNearFreePoint] The point is not in the map.\n" << endl;
        return false;
    }

    int x = point.x;
    int y = point.y;

    int max_extend = (extend_count == -1) ? max({x, gridSize.x - x, y, gridSize.y - y}) : extend_count;
    for (int i = 2; i < max_extend;)
    {
        for (int n = -1 * i + 1; n < i; n++)
        {
            point.x = x + n;
            point.y = y + i;
            if (!detectCollision(point))
                return true;

            point.x = x + n;
            point.y = y - i;
            if (!detectCollision(point))
                return true;

            point.x = x + i;
            point.y = y + n;
            if (!detectCollision(point))
                return true;

            point.x = x - i;
            point.y = y + n;
            if (!detectCollision(point))
                return true;
        }
        point.x = x + i;
        point.y = y + i;
        if (!detectCollision(point))
            return true;

        point.x = x + i;
        point.y = y - i;
        if (!detectCollision(point))
            return true;

        point.x = x - i;
        point.y = y + i;
        if (!detectCollision(point))
            return true;

        point.x = x - i;
        point.y = y - i;
        if (!detectCollision(point))
            return true;

        i += 2;
    }

    // find no free place, restore operation.
    point.x = x;
    point.y = y;
    return false;
}

bool baseMap::reprocessTarget(const Vec2d &start_, Vec2d &target_)
{
    if (inMap(target_))
    {
        return findNearFreePoint(target_);
    }
    else
    {
        // The given target is not in the map.

        // Find new target point in {border_free_points}.
        auto startVec = Eigen::Vector2i(start_.x, start_.y);
        auto targetVec = Eigen::Vector2i(target_.x, target_.y);
        auto s2t_vec = targetVec - startVec;
        double maxCos_theta = -1.1;
        int maxCos_index;

        auto vecSize = border_free_points.size();
        for (int i = 0; i < vecSize; i++)
        {
            double cosTheta = (s2t_vec.dot(border_free_points[i] - startVec) * 1.0) /
                              (s2t_vec.norm() * (border_free_points[i] - startVec).norm());
            if (cosTheta > maxCos_theta)
            {
                maxCos_theta = cosTheta;
                maxCos_index = i;
            }
        }
        if (maxCos_theta >= 0)
        {
            // Found a proper local target point in map border.
            target_.x = border_free_points[maxCos_index][0];
            target_.y = border_free_points[maxCos_index][1];
            return true;
        }

        // Go on if no proper target found.

        // Calculate the crossover point of s2t_vec and border.
        // If the start point is on border, fine tune it.
        auto newStartVec = startVec;
        // promise the start point not on map's border.
        if (start_.x == 0)
            newStartVec[0] += 1;
        else if (start_.x == gridSize.x)
            newStartVec[0] -= 1;
        if (start_.y == 0)
            newStartVec[1] += 1;
        else if (start_.y == gridSize.y)
            newStartVec[1] -= 1;

        // Four vector from start point to four corner point
        // (below_left(BL): {0, 0};           below_right(BR): {gridSize.x, 0};
        //  up_left(UL):    {0, gridSize.y};  up_right(UR):    {gridSize.x, gridSize.y})
        Eigen::Vector2i s2BL_vec = Eigen::Vector2i(0, 0) - newStartVec;
        Eigen::Vector2i s2BR_vec = Eigen::Vector2i(gridSize.x, 0) - newStartVec;
        Eigen::Vector2i s2UL_vec = Eigen::Vector2i(0, gridSize.y) - newStartVec;
        Eigen::Vector2i s2UR_vec = Eigen::Vector2i(gridSize.x, gridSize.y) - newStartVec;

        // Infer which area(up, right, below, left) the s2t_vec belongs to.
        // Judge clockwise, TRUE if s2t_vec is in clockwise direction of a s2[%Corner]_vec.
        // Calculate the cross point. (X - Xs) * Ys_t = (Y - Ys) * Xs_t.
        Vec2d crossPoint;
        bool rel2UL = s2t_vec[0] * s2UL_vec[1] - s2t_vec[1] * s2UL_vec[0];
        bool rel2UR = s2t_vec[0] * s2UR_vec[1] - s2t_vec[1] * s2UR_vec[0];
        if (rel2UL && (!rel2UR))
        {
            // s2t_vec belongs to UP area. Y = gridSize.y
            crossPoint.y = gridSize.y;
            crossPoint.x = (int) ((crossPoint.y - start_.y) * s2t_vec[0] / (1.0 * s2t_vec[1]) + start_.x);
            return findNearFreePoint(crossPoint);
        }
        bool rel2BR = s2t_vec[0] * s2BR_vec[1] - s2t_vec[1] * s2BR_vec[0];
        if (rel2UR && (!rel2BR))
        {
            // s2t_vec belongs to RIGHT area. X = gridSize.x
            crossPoint.x = gridSize.x;
            crossPoint.y = (int) ((crossPoint.x - start_.x) * s2t_vec[1] / (1.0 * s2t_vec[0]) + start_.y);
            return findNearFreePoint(crossPoint);
        }
        bool rel2BL = s2t_vec[0] * s2BL_vec[1] - s2t_vec[1] * s2BL_vec[0];
        if (rel2BL && (!rel2UL))
        {
            // s2t_vec belongs to LEFT area. X = 0
            crossPoint.x = 0;
            crossPoint.y = (int) ((crossPoint.x - start_.x) * s2t_vec[1] / (1.0 * s2t_vec[0]) + start_.y);
            return findNearFreePoint(crossPoint);
        }
        if (rel2BR && (!rel2BL))
        {
            // s2t_vec belongs to BELOW area. Y = 0
            crossPoint.y = 0;
            crossPoint.x = (int) ((crossPoint.y - start_.y) * s2t_vec[0] / (1.0 * s2t_vec[1]) + start_.x);
            return findNearFreePoint(crossPoint);
        }
    }
}

Planning::Planning()
{
    baseMap();
    moveDirections = {
            {0,  1},
            {1,  0},
            {0,  -1},
            {-1, 0},
            {-1, -1},
            {1,  1},
            {-1, 1},
            {1,  -1}}; //, {0, 0}
    //	NodeID = {{nullptr}};
}

void Planning::setGridMap(const VecVec &GridState_)
{
    gridSize = {int(GridState_.size()), int(GridState_[0].size())};
    GridState = DistanceMap(GridState_);
}

double Planning::calHeurCost(Vec2d startpoint_, Vec2d endpoint_)
{
    /*double deltaX, deltaY, ValueXmax, ValueYmax, ValueH, biasH;
    deltaX = abs(startpoint_.x - endpoint_.x);
    deltaY = abs(startpoint_.y - endpoint_.y);
    ValueXmax = deltaX + 0.414 * deltaY;
    ValueYmax = deltaY + 0.414 * deltaX;
    if (deltaX == 0 || deltaY == 0 || deltaX == deltaY)
    {
        biasH = 0;
    }
    else
    {
        biasH = 1.5;
    }
    ValueH = (deltaX > deltaY) ? ValueXmax : ValueYmax;
    return (ValueH + biasH);
    */
    return ((double) sqrt(double(startpoint_.x - endpoint_.x) * double(startpoint_.x - endpoint_.x) +
                          double(startpoint_.y - endpoint_.y) * double(startpoint_.y - endpoint_.y)));
}

void Planning::releaseNodes(NodeSet &nodes_)
{
    for (auto it = nodes_.begin(); it != nodes_.end();)
    {
        delete *it;
        it = nodes_.erase(it);
    }
}

CoordinateList Planning::findPath(Vec2d source, Vec2d target, Vec2d init_direction, int init_len, double distance_threshold, bool raw)
{
    /// Do not start planning if the distance between source and target is smaller than distance_threshold.
    if ((abs(target.x - source.x) < distance_threshold) && (abs(target.y - source.y) < distance_threshold))
        return {source};

    auto source_ = source;
    auto target_ = target;
    if (detectCollision(source_))
    {
        std::cout << "Source is invalid" << endl;
        if (findNearFreePoint(source_, 15))
        {
//            std::cout << "New Source: ( " << source_.x << ", " << source_.y << " )" << std::endl;
        }
        else
            return {source};
    }

    bool findTarget = reprocessTarget(source_, target_);
    if (!findTarget)
    {
        std::cout << "No target is found" << endl;
        return {source_};
    }

//    if (detectCollision(target_)) {
//        std::cout << "Target is invalid" << endl;
//        return {source_};
//    }

    Vec2d source_fix = source_;
    for (int i = 0; i < init_len; i++)
    {
        Vec2d tmp_source = source_fix + init_direction;
        if (detectCollision(tmp_source))
        {
            break;
        }
        source_fix = tmp_source;
    }

    Node *current = nullptr;
    NodeSet openSet, closedSet;

    VecVec VisitVec;
    VecVecIP NodeID;

    VisitVec.resize(gridSize.x);
    for (auto &elem: VisitVec)
    {
        elem.resize(int(gridSize.y), 0);
    }

    NodeID.resize(gridSize.x);
    for (auto &elem_: NodeID)
    {
        elem_.resize(gridSize.y, nullptr);
    }

    Node *tmpNode = new Node(source_fix);
    tmpNode->parent = tmpNode;
    openSet.insert(tmpNode);
    VisitVec[source_fix.x][source_fix.y] = 1; //put in OpenSet
    NodeID[source_fix.x][source_fix.y] = tmpNode;

    while (!openSet.empty())
    {
        current = *openSet.begin();

        if (current->coordinate == target_)
        {
            std::cout << "The path is found successfully, as follows." << endl;
            break;
        }

        openSet.erase(openSet.begin());
        closedSet.insert(current);
        VisitVec[current->coordinate.x][current->coordinate.y] = 2; //put in ClosedSet

        for (int i = 0; i < 8; ++i)
        {
            Vec2d newCoordinate(current->coordinate + moveDirections[i]);

            int newX = newCoordinate.x;
            int newY = newCoordinate.y;

            if (detectCollision(newCoordinate) ||
                VisitVec[newX][newY] == 2)
            {
                continue;
            }

            /*			if (i > 3)
            {
                if (!reachCorner(current->coordinate, newCoordinate))
                {
                    continue;
                }
            }
*/
            double turn = (current->coordinate + current->coordinate == (current->parent)->coordinate + newCoordinate)
                          ? 0.0 : astar_cost_factors.turn;
            double bias = (i < 4) ? 1.0 : astar_cost_factors.move_diag;
//            double distance_obstacle = 20.0 / (GridState[newX][newY] > (robot_block_radius + 5) ? 100000 : GridState[newX][newY]);
            double distance_obstacle;
            if (GridState[newX][newY] > (robot_block_radius + astar_cost_factors.max_distant_grid))
            {
                distance_obstacle = 0.0002;
            }
            else
            {
                distance_obstacle = astar_cost_factors.weight_distant *
                                    exp((robot_block_radius + 1.0 - GridState[newX][newY]) / 2.0);
            }
            double newG = current->G + bias + (raw ? 0 : (turn + distance_obstacle));
            double Wg = astar_cost_factors.weight_g;
            double Wf = astar_cost_factors.weight_f;

            if (VisitVec[newX][newY] == 0)
            {
                Node *successor = new Node(newCoordinate, current);
                successor->G = newG;
                successor->H = calHeurCost(successor->coordinate, target_);
                successor->F = Wg * successor->G + Wf * successor->H;
                //successor->p2cOrder = i;

                openSet.insert(successor);
                VisitVec[newX][newY] = 1;
                NodeID[newX][newY] = successor;

                /*				NodeID[newX][newY] = new Node(newCoordinate, current);
                NodeID[newX][newY]->G = newG;
                NodeID[newX][newY]->H = calHeurCost(newCoordinate, target_);
                NodeID[newX][newY]->F = 0.4*newG + 0.6*NodeID[newX][newY]->H;

                openSet.insert(NodeID[newX][newY]);
                VisitVec[newX][newY] = 1;*/
            }
            else if (newG < NodeID[newX][newY]->G)
            {
                openSet.erase(NodeID[newX][newY]);
                Node *successor = new Node(newCoordinate, current);
                successor->G = newG;
                successor->H = calHeurCost(successor->coordinate, target_);
                successor->F = Wg * successor->G + Wf * successor->H;
                //successor->p2cOrder = i;

                openSet.insert(successor);
                NodeID[newX][newY] = successor;

                /*				NodeID[newX][newY] = new Node(newCoordinate, current);
                NodeID[newX][newY]->G = newG;
                NodeID[newX][newY]->H = calHeurCost(newCoordinate, target_);
                NodeID[newX][newY]->F = 0.4*newG + 0.6*NodeID[newX][newY]->H;

                openSet.insert(NodeID[newX][newY]);
                VisitVec[newX][newY] = 1;*/
            }
        }
    }

    CoordinateList path;

    if (!(current->coordinate == target_))
    {
        path.push_back(source_);
        path.push_back(source_fix);
        std::cout << "There is no path towards the target!" << std::endl;
        return path;
    }

    while (current != tmpNode)
    {
        path.push_back(current->coordinate);
        current = current->parent;
    }
    path.push_back(source_fix);

    while (!(path.back() == source_))
    {
        path.push_back(path.back() - init_direction);
    }


    /*	Vec2d ii = current->coordinate;
    while (!(ii == source_fix))
    {
        path.push_back(ii);
        ii = NodeID[ii.x][ii.y]->parent;
    }
    path.push_back(source_fix);
*/

    reverse(path.begin(), path.end());

    releaseNodes(openSet);
    releaseNodes(closedSet);
/*
	int numO = 0;
	int numC = 0;
	for (int ii = 0; ii < gridSize.x; ++ii)
	{
		for (int iii = 0; iii < gridSize.x; ++iii)
		{
			if (VisitVec[ii][iii] == 1)
			{
				numO += 1;
			}
			else if (VisitVec[ii][iii] == 2)
			{
				numC += 1;
			}
		}
	}

	std::cout << "OpenPointNumber: " << numO << '\t'
			  << "ClosedPointNumber: " << numC << endl;
*/
    return path;
}

CoordinateList Planning::simplePath(Vec2d source_, Vec2d target_, Vec2d init_direction, int init_len, double distance_threshold)
{
    CoordinateList path_ = findPath(source_, target_, init_direction, init_len, distance_threshold);

    CoordinateList tmpPath, simpPath;
    /*	simpPath.push_back(path_[0]);
    if (path_.size() > 1)
    {
        int ii;
        for (ii = 1; ii < path_.size() - 1; ++ii)
        {
            if (NodeID[path_[ii].x][path_[ii].y]->p2cOrder !=
                NodeID[path_[ii + 1].x][path_[ii + 1].y]->p2cOrder)
            {
                simpPath.push_back(path_[ii]);
            }
            else
            {
                continue;
            }
        }
        simpPath.push_back(path_[ii]);
    }*/

    tmpPath.push_back(path_[0]);

    //simpAgain.push_back(path_[0]);
    for (int i = 1; i < path_.size() - 1; ++i)
    {
        if (2 * path_[i].x - path_[i - 1].x - path_[i + 1].x != 0 ||
            2 * path_[i].y - path_[i - 1].y - path_[i + 1].y != 0 ||
            abs(path_[i].x - tmpPath.back().x) > 6000 ||
            abs(path_[i].y - tmpPath.back().y) > 6000)
        {
            tmpPath.push_back(path_[i]);
        }
        else
        {
            continue;
        }
    }

    if (path_.size() > 1)
    {
        tmpPath.push_back(path_.back());
    }
    if (tmpPath.size() <= 3)
    {
        return tmpPath;
    }

    simpPath.push_back(tmpPath[0]);
    simpPath.push_back(tmpPath[1]);
    for (int j = 2; j < tmpPath.size() - 1; ++j)
    {
        pointCoord start(simpPath.back());
        pointCoord end(tmpPath[j + 1]);
        if (lineCollision(start, end))
        {
            simpPath.push_back(tmpPath[j]);
        }
        else
        {
            continue;
        }
    }
    simpPath.push_back(tmpPath.back());


/*	if (tmpPath.size() > 1)
	{
		for (int ii = 1; ii < tmpPath.size() - 1; ++ii)
		{
			if (abs(tmpPath[ii].x - tmpPath[ii + 1].x) > 3 ||
				abs(tmpPath[ii].y - tmpPath[ii + 1].y) > 3)
			{
				simpPath.push_back(tmpPath[ii]);
			}
		}
		simpPath.push_back(tmpPath.back());
	}

	if (simpPath.size() > 1)
	{
		for (int iii = 1; iii < simpPath.size() - 1; ++iii)
		{
			double k12 = (double(simpPath[iii].y - simpAgain.back().y)) /
							 (double(simpPath[iii].x - simpAgain.back().x)) +
						 0.000001;
			double k13 = (double(simpPath[iii + 1].y - simpAgain.back().y)) /
							 (double(simpPath[iii + 1].x - simpAgain.back().x)) +
						 0.000001;
			if (k12 * k13 < 0)
			{
				simpAgain.push_back(simpPath[iii]);
			}
			else if (k12 > 0 && k13 > 0)
			{
				if ((1 - min(k12, k13) / max(k12, k13)) > 0.3)
				{
					simpAgain.push_back(simpPath[iii]);
				}
			}
			else
			{
				if ((1 - max(k12, k13) / min(k12, k13)) > 0.3)
				{
					simpAgain.push_back(simpPath[iii]);
				}
			}
		}
		simpAgain.push_back(simpPath.back());
	}
*/
    return simpPath;
}

void Planning::showPath(CoordinateList path_)
{
    for (auto &coordinate_: path_)
    {
        std::cout << coordinate_.x << " " << coordinate_.y << endl;
    }
}

void Planning::setAstarCost(astar_cost astar_cost_factors_)
{
    astar_cost_factors = astar_cost_factors_;
}

void Planning::setAstarCost(float weight_g_, float weight_f_, float turn_, float move_diag_, float weight_distant_,
                            int max_distant_grid_)
{
    astar_cost_factors.weight_g = weight_g_;
    astar_cost_factors.weight_f = weight_f_;
    astar_cost_factors.turn = turn_;
    astar_cost_factors.move_diag = move_diag_;
    astar_cost_factors.weight_distant = weight_distant_;
    astar_cost_factors.max_distant_grid = max_distant_grid_;
}

BSpline::BSpline(CoordinateList path_, int k_)
{
    path = path_;
    k = k_;
    n = path_.size() - 1;
    int m = n + k + 1;
    u.resize(m + 1);
    for (int ii = 0; ii <= k; ++ii)
    {
        u[ii] = 0.0;
    }
    for (int jj = m; jj >= n + 1; --jj)
    {
        u[jj] = 1.0; //+ (jj - n - 1) * 0.0001
    }
    for (int j = k + 1; j < n + 1; ++j)
    {
        u[j] = double(j - k) / (n + 1 - k);
    }
}

double BSpline::NU(int i_, int k_, double t_)
{
    float ans;
    if (k_ == 0)
    {
        if (t_ >= u[i_] && t_ < u[i_ + 1])
        {
            ans = 1.0;
        }
        else
        {
            ans = 0.0;
        }
    }
    else if (k_ > 0)
    {
        if (u[i_ + k_] != u[i_] && u[i_ + k_ + 1] == u[i_ + 1])
        {
            ans = (t_ - u[i_]) / (u[i_ + k_] - u[i_]) * NU(i_, k_ - 1, t_);
        }
        else if (u[i_ + k_] == u[i_] && u[i_ + k_ + 1] != u[i_ + 1])
        {
            ans = (u[i_ + k_ + 1] - t_) / (u[i_ + k_ + 1] - u[i_ + 1]) * NU(i_ + 1, k_ - 1, t_);
        }
        else if (u[i_ + k_] == u[i_] && u[i_ + k_ + 1] == u[i_ + 1])
        {
            ans = 0.0;
        }
        else
        {
            ans = ((t_ - u[i_]) / (u[i_ + k_] - u[i_]) * NU(i_, k_ - 1, t_)) +
                  ((u[i_ + k_ + 1] - t_) / (u[i_ + k_ + 1] - u[i_ + 1]) * NU(i_ + 1, k_ - 1, t_));
        }
    }

    return ans;
}

pointCoord::pointCoord(Vec2d p)
{
    x = p.x;
    y = p.y;
}

bool pointCoord::operator==(const pointCoord &coordinate_)
{
    return (x == coordinate_.x && y == coordinate_.y);
}

pointCoord pointCoord::operator+(const pointCoord &coordinate_)
{
    return {x + coordinate_.x, y + coordinate_.y};
}

pointCoord pointCoord::operator-(const pointCoord &coordinate_)
{
    return {x - coordinate_.x, y - coordinate_.y};
}

double pointCoord::operator*(const pointCoord &coordinate_)
{
    return (x * coordinate_.x + y * coordinate_.y);
}

pointCoord pointCoord::operator*(const double &b)
{
    return {x * b, y * b};
}


pointCoord BSpline::PU(double t)
{
    pointCoord p = {0.0, 0.0};
    for (int i = 0; i <= n; ++i)
    {
        p = p + path[i] * NU(i, k, t);
    }
    return p;
}

/*
Arc::Arc()
{
	gridSize = {1, 1};
	GridState = {{0}};
}
*/
arcInfo Arc::Line(pointCoord startCoord_, pointCoord endCoord_)
{
    arcInfo line;
    line.rotFlag = 0;
    line.lenR = sqrt((endCoord_ - startCoord_) * (endCoord_ - startCoord_));
    line.theta = -1;
    line.startCoord = startCoord_;
    line.endCoord = endCoord_;
    line.midCoord = pointCoord(-1.0, -1.0);
    line.centreCoord = pointCoord(-1.0, -1.0);

    return line;
}

arcInfo Arc::singleArc(pointCoord startCoord_, pointCoord midCoord_, pointCoord endCoord_, double Rd, bool detectC)
{
    if (Rd <= 0)
    {
        cout << "Input Error! The value of Rd must be positive!" << endl;
        exit(0);
    }
    arcInfo single_arc;
    pointCoord unit_s2c;
    pointCoord start2mid = midCoord_ - startCoord_;
    pointCoord mid2end = endCoord_ - midCoord_;
    double len_s2m = sqrt(start2mid * start2mid);
    double len_m2e = sqrt(mid2end * mid2end);
    double theta_ = acos((start2mid * mid2end) / (len_s2m * len_m2e));
    double cross_ = start2mid.x * mid2end.y - start2mid.y * mid2end.x;
    double len;
    pointCoord unit_s2m = start2mid * (1.0 / len_s2m);
    pointCoord unit_m2e = mid2end * (1.0 / len_m2e);

    if (cross_ == 0)
    {
        return Line(startCoord_, endCoord_);
    }
    else if (cross_ > 0)
    {
        single_arc.rotFlag = 1;
        unit_s2c = pointCoord(-1 * start2mid.y / len_s2m, start2mid.x / len_s2m);
    }
    else
    {
        single_arc.rotFlag = -1;
        unit_s2c = pointCoord(start2mid.y / len_s2m, -1 * start2mid.x / len_s2m);
    }

    single_arc.theta = theta_;
    single_arc.midCoord = midCoord_;

    if (len_s2m < len_m2e)
    {
        len = len_s2m;
        single_arc.lenR = len / tan(single_arc.theta / 2);
        if (Rd < single_arc.lenR)
        {
            single_arc.lenR = Rd;
            len = single_arc.lenR * tan(single_arc.theta / 2);
            single_arc.startCoord = midCoord_ - unit_s2m * len;
            single_arc.endCoord = midCoord_ + unit_m2e * len;
        }
        else
        {
            single_arc.startCoord = startCoord_;
            single_arc.endCoord = midCoord_ + unit_m2e * len;
        }
    }
    else
    {
        len = len_m2e;
        single_arc.lenR = len / tan(single_arc.theta / 2);
        if (Rd < single_arc.lenR)
        {
            single_arc.lenR = Rd;
            len = single_arc.lenR * tan(single_arc.theta / 2);
            single_arc.startCoord = midCoord_ - unit_s2m * len;
            single_arc.endCoord = midCoord_ + unit_m2e * len;
        }
        else
        {
            single_arc.endCoord = endCoord_;
            single_arc.startCoord = midCoord_ - unit_s2m * len;
        }
    }

    single_arc.centreCoord = single_arc.startCoord + unit_s2c * single_arc.lenR;

    if (detectC)
    {
        while (arcCollision(single_arc))
        {
            single_arc.lenR = single_arc.lenR * 0.9;

            if (single_arc.lenR < 1)
            {
                return Line(startCoord_, midCoord_);
            }

            len = single_arc.lenR * tan(single_arc.theta / 2);
            single_arc.startCoord = single_arc.midCoord - unit_s2m * len;
            single_arc.endCoord = single_arc.midCoord + unit_m2e * len;
            single_arc.centreCoord = single_arc.startCoord + unit_s2c * single_arc.lenR;
        }
    }

    return single_arc;
}

bool Arc::point_in_arc(pointCoord p_, arcInfo arc_)
{
    pointCoord start2point = p_ - arc_.startCoord;
    pointCoord point2end = arc_.endCoord - p_;
    double cross_ = start2point.x * point2end.y - start2point.y * point2end.x;
    if (cross_ * arc_.rotFlag > 0)
    {
        return true;
    }
    return false;
}

bool Arc::arcCollision(arcInfo arc_)
{
    double centre_a = arc_.centreCoord.x;
    double centre_b = arc_.centreCoord.y;
    double r = arc_.lenR;
    double minX, maxX, minY, maxY;
    int nX, nY;

    if (arc_.startCoord.x < arc_.midCoord.x)
    {
        minX = arc_.startCoord.x;
        maxX = arc_.midCoord.x;
    }
    else
    {
        minX = arc_.midCoord.x;
        maxX = arc_.startCoord.x;
    }
    if (arc_.endCoord.x < minX)
    {
        minX = arc_.endCoord.x;
    }
    else if (arc_.endCoord.x > maxX)
    {
        maxX = arc_.endCoord.x;
    }

    if (arc_.startCoord.y < arc_.midCoord.y)
    {
        minY = arc_.startCoord.y;
        maxY = arc_.midCoord.y;
    }
    else
    {
        minY = arc_.midCoord.y;
        maxY = arc_.startCoord.y;
    }
    if (arc_.endCoord.y < minY)
    {
        minY = arc_.endCoord.y;
    }
    else if (arc_.endCoord.y > maxY)
    {
        maxY = arc_.endCoord.y;
    }

    nX = int(maxX + 0.5) - int(minX + 0.5);
    nY = int(maxY + 0.5) - int(minY + 0.5);
    for (int i = 0; i < nX; ++i)
    {
        double tmp_x = int(minX + 0.5) + 0.5 + i;
        double bias_y = sqrt(pow(r, 2) - pow(tmp_x - centre_a, 2));
        double tmp_y1 = centre_b + bias_y;
        double tmp_y2 = centre_b - bias_y;
        std::vector<pointCoord> tmpP;
        tmpP.push_back(pointCoord(tmp_x, tmp_y1));
        tmpP.push_back(pointCoord(tmp_x, tmp_y2));

        for (int j = 0; j < 2; ++j)
        {
            if (point_in_arc(tmpP[j], arc_))
            {
                if (crossPointCollision(tmpP[j]))
                {
                    return true;
                }

            }
            else
            {
                continue;
            }
        }
    }

    for (int ii = 0; ii < nY; ++ii)
    {
        double tmp_y = int(minY + 0.5) + 0.5 + ii;
        double bias_x = sqrt(pow(r, 2) - pow(tmp_y - centre_b, 2));
        double tmp_x1 = centre_a + bias_x;
        double tmp_x2 = centre_a - bias_x;
        std::vector<pointCoord> tmpPP;
        tmpPP.push_back(pointCoord(tmp_x1, tmp_y));
        tmpPP.push_back(pointCoord(tmp_x2, tmp_y));

        for (int jj = 0; jj < 2; ++jj)
        {
            if (point_in_arc(tmpPP[jj], arc_) && crossPointCollision(tmpPP[jj]))
            {
                return true;
            }
            else
            {
                continue;
            }
        }
    }

    return false;
}

VecVecD Arc::info2vec(const std::vector<arcInfo> &arc_)
{
    int len_ = arc_.size();
    VecVecD tmp_(len_);
    for (int i = 0; i < len_; ++i)
    {
        tmp_[i].push_back(arc_[i].rotFlag);
        tmp_[i].push_back(arc_[i].lenR);
        tmp_[i].push_back(arc_[i].startCoord.x);
        tmp_[i].push_back(arc_[i].startCoord.y);
        tmp_[i].push_back(arc_[i].endCoord.x);
        tmp_[i].push_back(arc_[i].endCoord.y);
        tmp_[i].push_back(arc_[i].theta);
        tmp_[i].push_back(arc_[i].centreCoord.x);
        tmp_[i].push_back(arc_[i].centreCoord.y);
        tmp_[i].push_back(arc_[i].midCoord.x);
        tmp_[i].push_back(arc_[i].midCoord.y);
    }

    return tmp_;
}

std::vector<arcInfo> Arc::ArcSmooth(CoordinateList path_, double Rd)
{
    std::vector<arcInfo> arc;
    int lenPath = path_.size();

    if (lenPath == 0)
    {
        cout << "Input Error! There is no point in the path!" << endl;
        exit(0);
    }

    if (lenPath == 1)
    {
        arcInfo tmp_ = {0, 0, 0, pointCoord(path_[0]), pointCoord(path_[0]), pointCoord(-1.0, -1.0),
                        pointCoord(-1.0, -1.0)};
        arc.push_back(tmp_);
        return arc;
    }

    if (lenPath == 2)
    {
        pointCoord start_(path_[0]);
        pointCoord end_(path_[1]);
        arc.push_back(Line(start_, end_));
        return arc;
    }

    if (Rd == 0)
    {
        for (int i = 0; i < lenPath - 1; ++i)
        {
            pointCoord start_(path_[i]);
            pointCoord end_(path_[i + 1]);
            arc.push_back(Line(start_, end_));
        }

        return arc;
    }

    if (lenPath == 3)
    {
        pointCoord start_(path_[0]);
        pointCoord mid_(path_[1]);
        pointCoord end_(path_[2]);
        arcInfo tmp_ = singleArc(start_, mid_, end_, Rd);
        if (!(tmp_.startCoord == start_))
        {
            arc.push_back(Line(start_, tmp_.startCoord));
        }
        arc.push_back(tmp_);
        if (!(tmp_.endCoord == end_))
        {
            arc.push_back(Line(tmp_.endCoord, end_));
        }
        return arc;
    }

    pointCoord start(path_[0]);

    for (int i = 1; i < lenPath - 2; ++i)
    {
        pointCoord mid_(path_[i]);
        pointCoord end_((path_[i] + path_[i + 1]) * 0.5);
        arcInfo tmpArc_ = singleArc(start, mid_, end_, Rd);
        if (!(tmpArc_.startCoord == start))
        {
            arc.push_back(Line(start, tmpArc_.startCoord));
        }
        arc.push_back(tmpArc_);
        start = tmpArc_.endCoord;
    }
    pointCoord mid(path_[lenPath - 2]);
    pointCoord end(path_[lenPath - 1]);
    arcInfo tmpArc = singleArc(start, mid, end, Rd);
    if (!(tmpArc.startCoord == start))
    {
        arc.push_back(Line(start, tmpArc.startCoord));
    }
    arc.push_back(tmpArc);
    if (!(tmpArc.endCoord == end))
    {
        arc.push_back(Line(tmpArc.endCoord, end));
    }

    for (int i = 1; i < arc.size() - 1; ++i)
    {
        if (arc[i].rotFlag != 0 && arc[i].lenR < Rd)
        {
            if (arc[i - 1].rotFlag == 0 && arc[i - 1].lenR != 0 && arc[i + 1].rotFlag == 0 && arc[i + 1].lenR != 0)
            {
                arc[i] = singleArc(arc[i - 1].startCoord, arc[i].midCoord, arc[i + 1].endCoord, Rd);
                arc[i - 1] = Line(arc[i - 1].startCoord, arc[i].startCoord);
                arc[i + 1] = Line(arc[i].endCoord, arc[i + 1].endCoord);
            }
        }
    }

    std::vector<arcInfo> fixed_arc;
    for (int i = 0; i < arc.size(); ++i)
    {
        if (arc[i].rotFlag == 0 && arc[i].lenR == 0)
        {
            continue;
        }
        else
        {
            fixed_arc.push_back(arc[i]);
        }
    }

    return fixed_arc;
}

void Arc::setGridMap(const VecVec &GridState_)
{
    gridSize = {int(GridState_.size()), int(GridState_[0].size())};
    GridState = DistanceMap(GridState_);
}

/*
ArcPlanning::ArcPlanning()
{
	baseMap::gridSize = {1, 1};
	GridState = {{0}};
	moveDirections = {{0, 1}, {1, 0}, {0, -1}, {-1, 0}, {-1, -1}, {1, 1}, {-1, 1}, {1, -1}};
}
*/
void ArcPlanning::setGridMap(const VecVec &GridState_)
{
    gridSize = {int(GridState_.size()), int(GridState_[0].size())};
    GridState = DistanceMap(GridState_);
}

std::vector<arcInfo>
ArcPlanning::arc_planning(int sourceX, int sourceY, int targetX, int targetY, int init_direcX, int init_direcY,
                          int init_len, double Rd, double distance_threshold)
{
    Vec2d source = {sourceX, sourceY};
    Vec2d target = {targetX, targetY};
    Vec2d init_direction = {init_direcX, init_direcY};

    auto simp_path = simplePath(source, target, init_direction, init_len, distance_threshold);
    auto smooth_path = ArcSmooth(simp_path, Rd);
    //auto final_path = info2vec(smooth_path);

    return smooth_path;
}

// vel(m/s), w(rad/s) // cmd: time(s), Vx(m/s), Vy(m/s), W(rad/s)
std::vector<std::vector<float>> AStar::path2cmd(const std::vector<arcInfo> &path_, float resolution, float vel, float w)
{
    std::vector<std::vector<float>> cmd;
    std::vector<float> tmpCmd(4);

    for (auto path: path_)
    {
        auto len_R = resolution * path.lenR;

        if (path.rotFlag == 0)
        {
            tmpCmd[0] = (float) (len_R / vel);
            tmpCmd[1] = vel;
            tmpCmd[2] = 0;
            tmpCmd[3] = 0;
        }
        else
        {
            if (len_R < 1)
            {
                tmpCmd[0] = (float) (path.theta / w);
                tmpCmd[1] = 0;
                tmpCmd[2] = 0;
                tmpCmd[3] = w * (float) path.rotFlag;
            }
            else
            {
                auto tmp_w = (float) (vel / len_R);
                if (tmp_w > w)
                {
                    tmpCmd[3] = w * (float) path.rotFlag;
                    tmpCmd[1] = (float) (w * len_R);
                    tmpCmd[2] = 0;
                    tmpCmd[0] = (float) (path.theta / w);
                }
                else
                {
                    tmpCmd[0] = (float) (len_R * path.theta / vel);
                    tmpCmd[1] = vel;
                    tmpCmd[2] = 0;
                    tmpCmd[3] = tmp_w * (float) path.rotFlag;
                }
            }
        }
        cmd.push_back(tmpCmd);
    }
    return cmd;
}

std::vector<std::vector<float>>
AStar::getPathCommand(const VecVec &gridVector, int sourceX, int sourceY, int targetX, int targetY,
                      int robot_block_radius_, int init_direcX, int init_direcY, int init_len, double Rd,
                      float resolution, float vel, float w)
{
//    sourceX += planning_cfg.get_lidar_pos_bias_grid()[0];
//    sourceY += planning_cfg.get_lidar_pos_bias_grid()[1];
//
//    if (!planning_cfg.get_target_follow())
//    {
//        // TODO calculate the target coordinates.
//
//
//    }
//
//    auto init_direcX = planning_cfg.get_init_direction()[0];
//    auto init_direcY = planning_cfg.get_init_direction()[1];
//    auto init_len = planning_cfg.get_default_forward_distance_grid();
//    auto Rd = planning_cfg.get_default_curve_radius_grid();

    ArcPlanning ArcSmoothPath(gridVector, robot_block_radius_);//planning_cfg.get_self_block_radius_grid()
    auto smooth_path = ArcSmoothPath.arc_planning(sourceX, sourceY, targetX, targetY, init_direcX, init_direcY,
                                                  init_len, Rd);

//    auto resolution = planning_cfg.get_map_grid_resolution();
//    auto vel = planning_cfg.get_default_vel();
//    auto w = planning_cfg.get_max_angular_vel();

    auto cmd = path2cmd(smooth_path, resolution, vel, w);
    return cmd;
}

VecVecD AStar::path_planning(VecVec gridVector, int sourceX, int sourceY, int targetX, int targetY, int init_direcX,
                             int init_direcY, int init_len, double Rd)
{
//	Vec2d source = {sourceX, sourceY};
//	Vec2d target = {targetX, targetY};
//	Vec2d init_direction = {init_direcX, init_direcY};

    ArcPlanning ArcSmoothPath(gridVector);
//	ArcSmoothPath.setGridMap(gridVector);
//	auto orig_path = smoothPath.findPath(source, target, init_direction, init_len);
//	auto simp_path = ArcSmoothPath.simplePath(source, target, init_direction, init_len);
//	smoothPath.showPath(simp_path);
//	auto smooth_path = ArcSmoothPath.ArcSmooth(simp_path, Rd);
    auto smooth_path = ArcSmoothPath.arc_planning(sourceX, sourceY, targetX, targetY, init_direcX, init_direcY,
                                                  init_len, Rd);
    auto final_path = ArcSmoothPath.info2vec(smooth_path);

    return final_path;
}

std::vector<std::vector<int>>
AStar::raw_path(VecVec gridVector, int sourceX, int sourceY, int targetX, int targetY, int init_direcX, int init_direcY,
                int init_len, bool raw)
{
    ArcPlanning rawPlanning(gridVector);
    Vec2d source = {sourceX, sourceY};
    Vec2d target = {targetX, targetY};
    Vec2d init_direction = {init_direcX, init_direcY};
    auto raw_path = rawPlanning.findPath(source, target, init_direction, init_len, raw);
    std::vector<std::vector<int>> path;

    for (auto &elem: raw_path)
    {
        std::vector<int> tmp;
        tmp.push_back(elem.x);
        tmp.push_back(elem.y);
        path.push_back(tmp);
    }

    return path;
}

void AStar::print_path(VecVecD path)
{
    cout << endl;
    cout << setw(10) << "flag";
    cout << setw(10) << "len/r";
    cout << setw(10) << "startX";
    cout << setw(10) << "startY";
    cout << setw(10) << "endX";
    cout << setw(10) << "endY";
    cout << setw(10) << "theta";
    cout << setw(10) << "centreX";
    cout << setw(10) << "centreY";
    cout << setw(10) << "midX";
    cout << setw(10) << "midY";
    cout << endl << endl;
    for (auto line: path)
    {
        for (int i = 0; i < line.size(); ++i)
        {
            cout << setw(10) << fixed << setprecision(2) << line[i];
        }
        cout << endl;
    }
    cout << "traj_Num: " << path.size() << endl;
}

std::vector<pointCoord> AStar::path2points(std::vector<arcInfo> path_, float deltaLen)
{
    std::vector<pointCoord> points;
    pointCoord c2s;
    pointCoord tmpP;
    float phi;
    float the;

    for (auto path: path_)
    {
        points.push_back(path.startCoord);
        if (path.rotFlag != 0)
        {
            c2s = path.startCoord - path.centreCoord;
            phi = path.rotFlag * deltaLen / path.lenR;
            the = phi;
            while (abs(the) < path.theta)
            {
                tmpP.x = cos(the) * c2s.x - sin(the) * c2s.y;
                tmpP.y = sin(the) * c2s.x + cos(the) * c2s.y;
                tmpP = tmpP + path.centreCoord;
                points.push_back(tmpP);

                the += phi;
            }
        }
    }
    points.push_back(path_.back().endCoord);

    return points;
}

std::vector<pointCoord> AStar::getPathPoints(VecVec gridVector, int sourceX, int sourceY, int targetX, int targetY,
                                             int init_direcX, int init_direcY, int init_len, double Rd, float deltaLen)
{
    ArcPlanning ArcSmoothPath(gridVector);
    auto smooth_path = ArcSmoothPath.arc_planning(sourceX, sourceY, targetX, targetY, init_direcX, init_direcY,
                                                  init_len, Rd);
    auto points = path2points(smooth_path, deltaLen);
    return points;
}
