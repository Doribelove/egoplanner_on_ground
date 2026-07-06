#include "bspline_opt/bspline_optimizer.h"
#include "bspline_opt/gradient_descent_optimizer.h"
#include <ros/ros.h>

namespace ego_planner
{

    void BsplineOptimizer::setParam(ros::NodeHandle &nh)
    {
        nh.param("optimization/lambda_smooth", lambda1_, -1.0);
        nh.param("optimization/lambda_collision", lambda2_, -1.0); // 注意：原版参数名可能叫 lambda_dist 或 lambda_collision，保持你代码里的原样
        nh.param("optimization/lambda_feasibility", lambda3_, -1.0);
        nh.param("optimization/lambda_fitness", lambda4_, -1.0);

        nh.param("optimization/dist0", dist0_, -1.0);
        nh.param("optimization/max_vel", max_vel_, -1.0);
        nh.param("optimization/max_acc", max_acc_, -1.0);

        nh.param("optimization/order", order_, 3);
    }

    void BsplineOptimizer::setEnvironment(const GridMap::Ptr &env)
    {
        this->grid_map_ = env;
    }

    void BsplineOptimizer::setControlPoints(const Eigen::MatrixXd &points)
    {
        cps_.points = points;
    }

    void BsplineOptimizer::setBsplineInterval(const double &ts) { bspline_interval_ = ts; }

    /* This function is very similar to check_collision_and_rebound(). */
    std::vector<std::vector<Eigen::Vector3d>> BsplineOptimizer::initControlPoints(Eigen::MatrixXd &init_points, bool flag_first_init /*= true*/)
    {
        Eigen::Vector2d temp_pos2d;
        Eigen::Vector3d temp_pos3d;
        if (flag_first_init)
        {
            cps_.clearance = dist0_;
            cps_.resize(init_points.cols());
            cps_.points = init_points;
        }

        /*** Segment the initial trajectory according to obstacles ***/
        constexpr int ENOUGH_INTERVAL = 2;
        double step_size = grid_map_->getResolution() / ((init_points.col(0) - init_points.rightCols(1)).norm() / (init_points.cols() - 1)) / 2;
        int in_id, out_id;
        vector<std::pair<int, int>> segment_ids;
        int same_occ_state_times = ENOUGH_INTERVAL + 1;
        bool occ, last_occ = false;
        bool flag_got_start = false, flag_got_end = false, flag_got_end_maybe = false;
        int i_end = (int)init_points.cols() - order_ - ((int)init_points.cols() - 2 * order_) / 3; 

        for (int i = order_; i <= i_end; ++i)
        {
            for (double a = 1.0; a >= 0.0; a -= step_size)
            {
                temp_pos3d = a * init_points.col(i - 1) + (1 - a) * init_points.col(i);
                temp_pos2d<<temp_pos3d(0),temp_pos3d(1);
                occ = grid_map_->getInflateOccupancy2d(temp_pos2d);

                if (occ && !last_occ)
                {
                    if (same_occ_state_times > ENOUGH_INTERVAL || i == order_)
                    {
                        in_id = i - 1;
                        flag_got_start = true;
                    }
                    same_occ_state_times = 0;
                    flag_got_end_maybe = false; 
                }
                else if (!occ && last_occ)
                {
                    out_id = i;
                    flag_got_end_maybe = true;
                    same_occ_state_times = 0;
                }
                else
                {
                    ++same_occ_state_times;
                }

                if (flag_got_end_maybe && (same_occ_state_times > ENOUGH_INTERVAL || (i == (int)init_points.cols() - order_)))
                {
                    flag_got_end_maybe = false;
                    flag_got_end = true;
                }

                last_occ = occ;

                if (flag_got_start && flag_got_end)
                {
                    flag_got_start = false;
                    flag_got_end = false;
                    segment_ids.push_back(std::pair<int, int>(in_id, out_id));
                }
            }
        }

        /*** a star search ***/
        vector<vector<Eigen::Vector3d>> a_star_pathes;
        for (size_t i = 0; i < segment_ids.size(); ++i)
        {
            Eigen::Vector3d in(init_points.col(segment_ids[i].first)), out(init_points.col(segment_ids[i].second));
            Eigen::Vector2d in2d,out2d;
            in2d<<in(0),in(1);
            out2d<<out(0),out(1);
            if (a_star_->AstarSearch(/*(in-out).norm()/10+0.05*/ 0.1, in2d, out2d))
            {
                a_star_pathes.push_back(a_star_->getPath());
            }
            else
            {
                ROS_ERROR("a star error, force return!");
                return a_star_pathes;
            }
        }

        /*** calculate bounds ***/
        int id_low_bound, id_up_bound;
        vector<std::pair<int, int>> bounds(segment_ids.size());
        for (size_t i = 0; i < segment_ids.size(); i++)
        {
            if (i == 0) 
            {
                id_low_bound = order_;
                if (segment_ids.size() > 1)
                {
                    id_up_bound = (int)(((segment_ids[0].second + segment_ids[1].first) - 1.0f) / 2); 
                }
                else
                {
                    id_up_bound = init_points.cols() - order_ - 1;
                }
            }
            else if (i == segment_ids.size() - 1) 
            {
                id_low_bound = (int)(((segment_ids[i].first + segment_ids[i - 1].second) + 1.0f) / 2); 
                id_up_bound = init_points.cols() - order_ - 1;
            }
            else
            {
                id_low_bound = (int)(((segment_ids[i].first + segment_ids[i - 1].second) + 1.0f) / 2); 
                id_up_bound = (int)(((segment_ids[i].second + segment_ids[i + 1].first) - 1.0f) / 2);  
            }
            bounds[i] = std::pair<int, int>(id_low_bound, id_up_bound);
        }

        /*** Adjust segment length ***/
        vector<std::pair<int, int>> final_segment_ids(segment_ids.size());
        constexpr double MINIMUM_PERCENT = 0.0; 
        int minimum_points = round(init_points.cols() * MINIMUM_PERCENT), num_points;
        for (size_t i = 0; i < segment_ids.size(); i++)
        {
            num_points = segment_ids[i].second - segment_ids[i].first + 1;
            if (num_points < minimum_points)
            {
                double add_points_each_side = (int)(((minimum_points - num_points) + 1.0f) / 2);
                final_segment_ids[i].first = segment_ids[i].first - add_points_each_side >= bounds[i].first ? segment_ids[i].first - add_points_each_side : bounds[i].first;
                final_segment_ids[i].second = segment_ids[i].second + add_points_each_side <= bounds[i].second ? segment_ids[i].second + add_points_each_side : bounds[i].second;
            }
            else
            {
                final_segment_ids[i].first = segment_ids[i].first;
                final_segment_ids[i].second = segment_ids[i].second;
            }
        }

        /*** Assign data to each segment ***/
        for (size_t i = 0; i < segment_ids.size(); i++)
        {
            // step 1
            for (int j = final_segment_ids[i].first; j <= final_segment_ids[i].second; ++j)
                cps_.flag_temp[j] = false;

            // step 2
            int got_intersection_id = -1;
            for (int j = segment_ids[i].first + 1; j < segment_ids[i].second; ++j)
            {
                Eigen::Vector3d ctrl_pts_law(cps_.points.col(j + 1) - cps_.points.col(j - 1)), intersection_point;
                int Astar_id = a_star_pathes[i].size() / 2, last_Astar_id; 
                double val = (a_star_pathes[i][Astar_id] - cps_.points.col(j)).dot(ctrl_pts_law), last_val = val;
                while (Astar_id >= 0 && Astar_id < (int)a_star_pathes[i].size())
                {
                    last_Astar_id = Astar_id;

                    if (val >= 0) --Astar_id; else ++Astar_id;

                    val = (a_star_pathes[i][Astar_id] - cps_.points.col(j)).dot(ctrl_pts_law);

                    if (val * last_val <= 0 && (abs(val) > 0 || abs(last_val) > 0)) 
                    {
                        intersection_point =
                                a_star_pathes[i][Astar_id] +
                                ((a_star_pathes[i][Astar_id] - a_star_pathes[i][last_Astar_id]) *
                                 (ctrl_pts_law.dot(cps_.points.col(j) - a_star_pathes[i][Astar_id]) / ctrl_pts_law.dot(a_star_pathes[i][Astar_id] - a_star_pathes[i][last_Astar_id])) 
                                );
                        got_intersection_id = j;
                        break;
                    }
                }

                if (got_intersection_id >= 0)
                {
                    cps_.flag_temp[j] = true;
                    double length = (intersection_point - cps_.points.col(j)).norm();
                    if (length > 1e-5)
                    {
                        for (double a = length; a >= 0.0; a -= grid_map_->getResolution())
                        {
                            temp_pos3d = (a / length) * intersection_point + (1 - a / length) * cps_.points.col(j);
                            temp_pos2d<<temp_pos3d(0),temp_pos3d(1);
                            occ = grid_map_->getInflateOccupancy2d(temp_pos2d);

                            if (occ || a < grid_map_->getResolution())
                            {
                                if (occ) a += grid_map_->getResolution();
                                cps_.base_point[j].push_back((a / length) * intersection_point + (1 - a / length) * cps_.points.col(j));
                                cps_.direction[j].push_back((intersection_point - cps_.points.col(j)).normalized());
                                break;
                            }
                        }
                    }
                }
            }

            if (segment_ids[i].second - segment_ids[i].first == 1)
            {
                Eigen::Vector3d ctrl_pts_law(cps_.points.col(segment_ids[i].second) - cps_.points.col(segment_ids[i].first)), intersection_point;
                Eigen::Vector3d middle_point = (cps_.points.col(segment_ids[i].second) + cps_.points.col(segment_ids[i].first)) / 2;
                int Astar_id = a_star_pathes[i].size() / 2, last_Astar_id; 
                double val = (a_star_pathes[i][Astar_id] - middle_point).dot(ctrl_pts_law), last_val = val;
                while (Astar_id >= 0 && Astar_id < (int)a_star_pathes[i].size())
                {
                    last_Astar_id = Astar_id;
                    if (val >= 0) --Astar_id; else ++Astar_id;
                    val = (a_star_pathes[i][Astar_id] - middle_point).dot(ctrl_pts_law);

                    if (val * last_val <= 0 && (abs(val) > 0 || abs(last_val) > 0)) 
                    {
                        intersection_point =
                                a_star_pathes[i][Astar_id] +
                                ((a_star_pathes[i][Astar_id] - a_star_pathes[i][last_Astar_id]) *
                                 (ctrl_pts_law.dot(middle_point - a_star_pathes[i][Astar_id]) / ctrl_pts_law.dot(a_star_pathes[i][Astar_id] - a_star_pathes[i][last_Astar_id]))
                                );

                        if ((intersection_point - middle_point).norm() > 0.01) 
                        {
                            cps_.flag_temp[segment_ids[i].first] = true;
                            cps_.base_point[segment_ids[i].first].push_back(cps_.points.col(segment_ids[i].first));
                            cps_.direction[segment_ids[i].first].push_back((intersection_point - middle_point).normalized());
                            got_intersection_id = segment_ids[i].first;
                        }
                        break;
                    }
                }
            }

            if (got_intersection_id >= 0)
            {
                for (int j = got_intersection_id + 1; j <= final_segment_ids[i].second; ++j)
                    if (!cps_.flag_temp[j])
                    {
                        cps_.base_point[j].push_back(cps_.base_point[j - 1].back());
                        cps_.direction[j].push_back(cps_.direction[j - 1].back());
                    }

                for (int j = got_intersection_id - 1; j >= final_segment_ids[i].first; --j)
                    if (!cps_.flag_temp[j])
                    {
                        cps_.base_point[j].push_back(cps_.base_point[j + 1].back());
                        cps_.direction[j].push_back(cps_.direction[j + 1].back());
                    }
            }
        }

        return a_star_pathes;
    }

    int BsplineOptimizer::earlyExit(void *func_data, const double *x, const double *g, const double fx, const double xnorm, const double gnorm, const double step, int n, int k, int ls)
    {
        BsplineOptimizer *opt = reinterpret_cast<BsplineOptimizer *>(func_data);
        return (opt->force_stop_type_ == STOP_FOR_ERROR || opt->force_stop_type_ == STOP_FOR_REBOUND);
    }

    double BsplineOptimizer::costFunctionRebound(void *func_data, const double *x, double *grad, const int n)
    {
        BsplineOptimizer *opt = reinterpret_cast<BsplineOptimizer *>(func_data);
        double cost;
        opt->combineCostRebound(x, grad, cost, n);
        opt->iter_num_ += 1;
        return cost;
    }

    double BsplineOptimizer::costFunctionRefine(void *func_data, const double *x, double *grad, const int n)
    {
        BsplineOptimizer *opt = reinterpret_cast<BsplineOptimizer *>(func_data);
        double cost;
        opt->combineCostRefine(x, grad, cost, n);
        opt->iter_num_ += 1;
        return cost;
    }

    // [保留原版]
    void BsplineOptimizer::calcDistanceCostRebound(const Eigen::MatrixXd &q, double &cost,
                                                   Eigen::MatrixXd &gradient, int iter_num, double smoothness_cost)
    {
        cost = 0.0;
        int end_idx = q.cols() - order_;
        double demarcation = cps_.clearance;
        double a = 3 * demarcation, b = -3 * pow(demarcation, 2), c = pow(demarcation, 3);

        force_stop_type_ = DONT_STOP;
        if (iter_num > 3 && smoothness_cost / (cps_.size - 2 * order_) < 0.1) 
        {
            check_collision_and_rebound();
        }

        for (auto i = order_; i < end_idx; ++i)
        {
            for (size_t j = 0; j < cps_.direction[i].size(); ++j)
            {
                double dist = (cps_.points.col(i) - cps_.base_point[i][j]).dot(cps_.direction[i][j]);
                double dist_err = cps_.clearance - dist;
                Eigen::Vector3d dist_grad = cps_.direction[i][j];

                if (dist_err < 0) { /* do nothing */ }
                else if (dist_err < demarcation)
                {
                    cost += pow(dist_err, 3);
                    gradient.col(i) += -3.0 * dist_err * dist_err * dist_grad;
                }
                else
                {
                    cost += a * dist_err * dist_err + b * dist_err + c;
                    gradient.col(i) += -(2.0 * a * dist_err + b) * dist_grad;
                }
            }
        }
    }

    // [保留原版]
    void BsplineOptimizer::calcFitnessCost(const Eigen::MatrixXd &q, double &cost, Eigen::MatrixXd &gradient)
    {
        cost = 0.0;
        int end_idx = q.cols() - order_;
        double a2 = 25, b2 = 1;
        for (auto i = order_ - 1; i < end_idx + 1; ++i)
        {
            Eigen::Vector3d x = (q.col(i - 1) + 4 * q.col(i) + q.col(i + 1)) / 6.0 - ref_pts_[i - 1];
            Eigen::Vector3d v = (ref_pts_[i] - ref_pts_[i - 2]).normalized();

            double xdotv = x.dot(v);
            Eigen::Vector3d xcrossv = x.cross(v);

            double f = pow((xdotv), 2) / a2 + pow(xcrossv.norm(), 2) / b2;
            cost += f;

            Eigen::Matrix3d m;
            m << 0, -v(2), v(1), v(2), 0, -v(0), -v(1), v(0), 0;
            Eigen::Vector3d df_dx = 2 * xdotv / a2 * v + 2 / b2 * m * xcrossv;

            gradient.col(i - 1) += df_dx / 6;
            gradient.col(i) += 4 * df_dx / 6;
            gradient.col(i + 1) += df_dx / 6;
        }
    }

    // [保留原版]
    void BsplineOptimizer::calcSmoothnessCost(const Eigen::MatrixXd &q, double &cost,
                                              Eigen::MatrixXd &gradient, bool falg_use_jerk /* = true*/)
    {
        cost = 0.0;
        if (falg_use_jerk)
        {
            Eigen::Vector3d jerk, temp_j;
            for (int i = 0; i < q.cols() - 3; i++)
            {
                jerk = q.col(i + 3) - 3 * q.col(i + 2) + 3 * q.col(i + 1) - q.col(i);
                cost += jerk.squaredNorm();
                temp_j = 2.0 * jerk;
                gradient.col(i + 0) += -temp_j;
                gradient.col(i + 1) += 3.0 * temp_j;
                gradient.col(i + 2) += -3.0 * temp_j;
                gradient.col(i + 3) += temp_j;
            }
        }
        else
        {
            Eigen::Vector3d acc, temp_acc;
            for (int i = 0; i < q.cols() - 2; i++)
            {
                acc = q.col(i + 2) - 2 * q.col(i + 1) + q.col(i);
                cost += acc.squaredNorm();
                temp_acc = 2.0 * acc;
                gradient.col(i + 0) += temp_acc;
                gradient.col(i + 1) += -2.0 * temp_acc;
                gradient.col(i + 2) += temp_acc;
            }
        }
    }

    // [核心修改点] calcFeasibilityCost
    // 替换了原有的逻辑，加入了阿克曼法向加速度约束
    void BsplineOptimizer::calcFeasibilityCost(const Eigen::MatrixXd &q, double &cost,
                                               Eigen::MatrixXd &gradient)
    {
        cost = 0.0;
        double ts = bspline_interval_;
        double ts_inv2 = 1 / ts / ts;

        // 阿克曼动力学参数
        double max_lat_acc = 0.8; // 限制转弯过猛 (决定了最小转弯半径)
        double max_lin_acc = max_acc_; 

        /* velocity feasibility */
        for (int i = 0; i < q.cols() - 1; i++)
        {
            Eigen::Vector3d vi = (q.col(i + 1) - q.col(i)) / ts;
            for (int j = 0; j < 3; j++)
            {
                if (vi(j) > max_vel_)
                {
                    cost += pow(vi(j) - max_vel_, 2) * ts_inv2;
                    gradient(j, i + 0) += -2 * (vi(j) - max_vel_) / ts * ts_inv2;
                    gradient(j, i + 1) += 2 * (vi(j) - max_vel_) / ts * ts_inv2;
                }
                else if (vi(j) < -max_vel_)
                {
                    cost += pow(vi(j) + max_vel_, 2) * ts_inv2;
                    gradient(j, i + 0) += -2 * (vi(j) + max_vel_) / ts * ts_inv2;
                    gradient(j, i + 1) += 2 * (vi(j) + max_vel_) / ts * ts_inv2;
                }
            }
        }

        /* acceleration feasibility (Ackermann constrained) */
        for (int i = 0; i < q.cols() - 2; i++)
        {
            Eigen::Vector3d ai = (q.col(i + 2) - 2 * q.col(i + 1) + q.col(i)) * ts_inv2;
            Eigen::Vector3d vi = (q.col(i + 1) - q.col(i)) / ts;
            double v_norm = vi.norm();

            if (v_norm > 0.1) // 运动时分解加速度
            {
                Eigen::Vector3d t_vec = vi / v_norm; 
                Eigen::Vector3d ai_tan = (ai.dot(t_vec)) * t_vec; // 切向分量
                Eigen::Vector3d ai_norm = ai - ai_tan;            // 法向分量

                // 惩罚切向加速度 (加减速)
                if (ai_tan.norm() > max_lin_acc) 
                {
                     cost += pow(ai_tan.norm() - max_lin_acc, 2);
                     Eigen::Vector3d g_now = 2 * (ai_tan.norm() - max_lin_acc) * ai_tan.normalized() * ts_inv2;
                     
                     gradient.col(i + 0) += g_now;
                     gradient.col(i + 1) += -2 * g_now;
                     gradient.col(i + 2) += g_now;
                }

                // [重点] 惩罚法向加速度 (转弯) -> 迫使转大弯
                if (ai_norm.norm() > max_lat_acc)
                {
                     cost += pow(ai_norm.norm() - max_lat_acc, 2) * 10.0; // 权重 x10
                     Eigen::Vector3d g_now = 2 * (ai_norm.norm() - max_lat_acc) * ai_norm.normalized() * ts_inv2 * 10.0;

                     gradient.col(i + 0) += g_now;
                     gradient.col(i + 1) += -2 * g_now;
                     gradient.col(i + 2) += g_now;
                }
            }
            else 
            {
                // 速度很低时，回退到普通限制 (逐轴检查)
                for (int j = 0; j < 3; j++)
                {
                    if (ai(j) > max_acc_)
                    {
                        cost += pow(ai(j) - max_acc_, 2);
                        gradient(j, i + 0) += 2 * (ai(j) - max_acc_) * ts_inv2;
                        gradient(j, i + 1) += -4 * (ai(j) - max_acc_) * ts_inv2;
                        gradient(j, i + 2) += 2 * (ai(j) - max_acc_) * ts_inv2;
                    }
                    else if (ai(j) < -max_acc_)
                    {
                        cost += pow(ai(j) + max_acc_, 2);
                        gradient(j, i + 0) += 2 * (ai(j) + max_acc_) * ts_inv2;
                        gradient(j, i + 1) += -4 * (ai(j) + max_acc_) * ts_inv2;
                        gradient(j, i + 2) += 2 * (ai(j) + max_acc_) * ts_inv2;
                    }
                }
            }
        }
    }

    bool BsplineOptimizer::check_collision_and_rebound(void)
    {
        Eigen::Vector3d temp_pos3d;
        Eigen::Vector2d temp_pos2d;
        int end_idx = cps_.size - order_;

        /*** Check and segment the initial trajectory according to obstacles ***/
        int in_id, out_id;
        vector<std::pair<int, int>> segment_ids;
        bool flag_new_obs_valid = false;
        int i_end = end_idx - (end_idx - order_) / 3;
        for (int i = order_ - 1; i <= i_end; ++i)
        {
            temp_pos3d = cps_.points.col(i);
            temp_pos2d<<temp_pos3d(0),temp_pos3d(1);
            bool occ = grid_map_->getInflateOccupancy2d(temp_pos2d);

            /*** check if the new collision will be valid ***/
            if (occ)
            {
                for (size_t k = 0; k < cps_.direction[i].size(); ++k)
                {
                    cout.precision(2);
                    if ((cps_.points.col(i) - cps_.base_point[i][k]).dot(cps_.direction[i][k]) < 1 * grid_map_->getResolution()) 
                    {
                        occ = false; 
                        break;
                    }
                }
            }

            if (occ)
            {
                flag_new_obs_valid = true;
                int j;
                for (j = i - 1; j >= 0; --j)
                {
                    temp_pos3d = cps_.points.col(j);
                    temp_pos2d<<temp_pos3d(0),temp_pos3d(1);
                    occ = grid_map_->getInflateOccupancy2d(temp_pos2d);
                    if (!occ) { in_id = j; break; }
                }
                if (j < 0) { in_id = 0; }

                for (j = i + 1; j < cps_.size; ++j)
                {
                    temp_pos3d = cps_.points.col(j);
                    temp_pos2d<<temp_pos3d(0),temp_pos3d(1);
                    occ = grid_map_->getInflateOccupancy2d(temp_pos2d);
                    if (!occ) { out_id = j; break; }
                }
                if (j >= cps_.size) 
                {
                    force_stop_type_ = STOP_FOR_ERROR;
                    return false;
                }

                i = j + 1;
                segment_ids.push_back(std::pair<int, int>(in_id, out_id));
            }
        }

        if (flag_new_obs_valid)
        {
            vector<vector<Eigen::Vector3d>> a_star_pathes;
            for (size_t i = 0; i < segment_ids.size(); ++i)
            {
                Eigen::Vector3d in(cps_.points.col(segment_ids[i].first)), out(cps_.points.col(segment_ids[i].second));
                Eigen::Vector2d in2d,out2d;
                in2d<<in(0),in(1);
                out2d<<out(0),out(1);
                if (a_star_->AstarSearch(0.1, in2d, out2d))
                {
                    a_star_pathes.push_back(a_star_->getPath());
                }
                else
                {
                    ROS_ERROR("a star error");
                    segment_ids.erase(segment_ids.begin() + i);
                    i--;
                }
            }

            for (size_t i = 0; i < segment_ids.size(); ++i)
            {
                for (int j = segment_ids[i].first; j <= segment_ids[i].second; ++j) cps_.flag_temp[j] = false;

                int got_intersection_id = -1;
                for (int j = segment_ids[i].first + 1; j < segment_ids[i].second; ++j)
                {
                    Eigen::Vector3d ctrl_pts_law(cps_.points.col(j + 1) - cps_.points.col(j - 1)), intersection_point;
                    int Astar_id = a_star_pathes[i].size() / 2, last_Astar_id; 
                    double val = (a_star_pathes[i][Astar_id] - cps_.points.col(j)).dot(ctrl_pts_law), last_val = val;
                    while (Astar_id >= 0 && Astar_id < (int)a_star_pathes[i].size())
                    {
                        last_Astar_id = Astar_id;
                        if (val >= 0) --Astar_id; else ++Astar_id;
                        val = (a_star_pathes[i][Astar_id] - cps_.points.col(j)).dot(ctrl_pts_law);

                        if (val * last_val <= 0 && (abs(val) > 0 || abs(last_val) > 0)) 
                        {
                            intersection_point =
                                    a_star_pathes[i][Astar_id] +
                                    ((a_star_pathes[i][Astar_id] - a_star_pathes[i][last_Astar_id]) *
                                     (ctrl_pts_law.dot(cps_.points.col(j) - a_star_pathes[i][Astar_id]) / ctrl_pts_law.dot(a_star_pathes[i][Astar_id] - a_star_pathes[i][last_Astar_id])) 
                                    );
                            got_intersection_id = j;
                            break;
                        }
                    }

                    if (got_intersection_id >= 0)
                    {
                        cps_.flag_temp[j] = true;
                        double length = (intersection_point - cps_.points.col(j)).norm();
                        if (length > 1e-5)
                        {
                            for (double a = length; a >= 0.0; a -= grid_map_->getResolution())
                            {
                                temp_pos3d = (a / length) * intersection_point + (1 - a / length) * cps_.points.col(j);
                                temp_pos2d<<temp_pos3d(0),temp_pos3d(1);
                                bool occ = grid_map_->getInflateOccupancy2d(temp_pos2d);

                                if (occ || a < grid_map_->getResolution())
                                {
                                    if (occ) a += grid_map_->getResolution();
                                    cps_.base_point[j].push_back((a / length) * intersection_point + (1 - a / length) * cps_.points.col(j));
                                    cps_.direction[j].push_back((intersection_point - cps_.points.col(j)).normalized());
                                    break;
                                }
                            }
                        }
                        else
                        {
                            got_intersection_id = -1;
                        }
                    }
                }

                if (got_intersection_id >= 0)
                {
                    for (int j = got_intersection_id + 1; j <= segment_ids[i].second; ++j)
                        if (!cps_.flag_temp[j])
                        {
                            cps_.base_point[j].push_back(cps_.base_point[j - 1].back());
                            cps_.direction[j].push_back(cps_.direction[j - 1].back());
                        }

                    for (int j = got_intersection_id - 1; j >= segment_ids[i].first; --j)
                        if (!cps_.flag_temp[j])
                        {
                            cps_.base_point[j].push_back(cps_.base_point[j + 1].back());
                            cps_.direction[j].push_back(cps_.direction[j + 1].back());
                        }
                }
            }

            force_stop_type_ = STOP_FOR_REBOUND;
            return true;
        }

        return false;
    }

    bool BsplineOptimizer::BsplineOptimizeTrajRebound(Eigen::MatrixXd &optimal_points, double ts)
    {
        setBsplineInterval(ts);
        bool flag_success = rebound_optimize();
        optimal_points = cps_.points;
        return flag_success;
    }

    bool BsplineOptimizer::BsplineOptimizeTrajRefine(const Eigen::MatrixXd &init_points, const double ts, Eigen::MatrixXd &optimal_points)
    {
        setControlPoints(init_points);
        setBsplineInterval(ts);
        bool flag_success = refine_optimize();
        optimal_points = cps_.points;
        return flag_success;
    }

    bool BsplineOptimizer::rebound_optimize()
    {
        Eigen::Vector3d temp_pos3d;
        Eigen::Vector2d temp_pos2d;
        iter_num_ = 0;
        int start_id = order_;
        int end_id = this->cps_.size - order_;
        variable_num_ = 3 * (end_id - start_id);
        double final_cost;

        ros::Time t0 = ros::Time::now(), t1, t2;
        int restart_nums = 0, rebound_times = 0;
        bool flag_force_return, flag_occ, success;
        new_lambda2_ = lambda2_;
        constexpr int MAX_RESART_NUMS_SET = 3;
        do
        {
            min_cost_ = std::numeric_limits<double>::max();
            iter_num_ = 0;
            flag_force_return = false;
            flag_occ = false;
            success = false;

            double q[variable_num_];
            memcpy(q, cps_.points.data() + 3 * start_id, variable_num_ * sizeof(q[0]));

            lbfgs::lbfgs_parameter_t lbfgs_params;
            lbfgs::lbfgs_load_default_parameters(&lbfgs_params);
            lbfgs_params.mem_size = 16;
            lbfgs_params.max_iterations = 200;
            lbfgs_params.g_epsilon = 0.01;

            t1 = ros::Time::now();
            int result = lbfgs::lbfgs_optimize(variable_num_, q, &final_cost, BsplineOptimizer::costFunctionRebound, NULL, BsplineOptimizer::earlyExit, this, &lbfgs_params);
            t2 = ros::Time::now();
            double time_ms = (t2 - t1).toSec() * 1000;
            double total_time_ms = (t2 - t0).toSec() * 1000;

            if (result == lbfgs::LBFGS_CONVERGENCE ||
                result == lbfgs::LBFGSERR_MAXIMUMITERATION ||
                result == lbfgs::LBFGS_ALREADY_MINIMIZED ||
                result == lbfgs::LBFGS_STOP)
            {
                flag_force_return = false;

                UniformBspline traj = UniformBspline(cps_.points, 3, bspline_interval_);
                double tm, tmp;
                traj.getTimeSpan(tm, tmp);
                double t_step = (tmp - tm) / ((traj.evaluateDeBoorT(tmp) - traj.evaluateDeBoorT(tm)).norm() / grid_map_->getResolution());
                for (double t = tm; t < tmp * 2 / 3; t += t_step) 
                {
                    temp_pos3d = traj.evaluateDeBoorT(t);
                    temp_pos2d <<temp_pos3d(0),temp_pos3d(1);
                    flag_occ = grid_map_->getInflateOccupancy2d(temp_pos2d);
                    if (flag_occ)
                    {
                        if (t <= bspline_interval_) 
                        {
                            return false;
                        }
                        break;
                    }
                }

                if (!flag_occ)
                {
                    success = true;
                }
                else 
                {
                    restart_nums++;
                    initControlPoints(cps_.points, false);
                    new_lambda2_ *= 2;
                }
            }
            else if (result == lbfgs::LBFGSERR_CANCELED)
            {
                flag_force_return = true;
                rebound_times++;
            }
            else
            {
                // ROS_WARN("Solver error");
            }

        } while ((flag_occ && restart_nums < MAX_RESART_NUMS_SET) ||
                 (flag_force_return && force_stop_type_ == STOP_FOR_REBOUND && rebound_times <= 20));

        return success;
    }

    bool BsplineOptimizer::refine_optimize()
    {
        Eigen::Vector3d temp_pos3d;
        Eigen::Vector2d temp_pos2d;
        iter_num_ = 0;
        int start_id = order_;
        int end_id = this->cps_.points.cols() - order_;
        variable_num_ = 3 * (end_id - start_id);

        double q[variable_num_];
        double final_cost;

        memcpy(q, cps_.points.data() + 3 * start_id, variable_num_ * sizeof(q[0]));

        double origin_lambda4 = lambda4_;
        bool flag_safe = true;
        int iter_count = 0;
        do
        {
            lbfgs::lbfgs_parameter_t lbfgs_params;
            lbfgs::lbfgs_load_default_parameters(&lbfgs_params);
            lbfgs_params.mem_size = 16;
            lbfgs_params.max_iterations = 200;
            lbfgs_params.g_epsilon = 0.001;

            int result = lbfgs::lbfgs_optimize(variable_num_, q, &final_cost, BsplineOptimizer::costFunctionRefine, NULL, NULL, this, &lbfgs_params);
            
            UniformBspline traj = UniformBspline(cps_.points, 3, bspline_interval_);
            double tm, tmp;
            traj.getTimeSpan(tm, tmp);
            double t_step = (tmp - tm) / ((traj.evaluateDeBoorT(tmp) - traj.evaluateDeBoorT(tm)).norm() / grid_map_->getResolution()); 
            for (double t = tm; t < tmp * 2 / 3; t += t_step)
            {
                temp_pos3d = traj.evaluateDeBoorT(t);
                temp_pos2d <<temp_pos3d(0),temp_pos3d(1);
                if (grid_map_->getInflateOccupancy2d(temp_pos2d))
                {
                    flag_safe = false;
                    break;
                }
            }

            if (!flag_safe)
                lambda4_ *= 2;

            iter_count++;
        } while (!flag_safe && iter_count <= 0);

        lambda4_ = origin_lambda4;
        return flag_safe;
    }

    void BsplineOptimizer::combineCostRebound(const double *x, double *grad, double &f_combine, const int n)
    {
        memcpy(cps_.points.data() + 3 * order_, x, n * sizeof(x[0]));

        double f_smoothness, f_distance, f_feasibility;

        Eigen::MatrixXd g_smoothness = Eigen::MatrixXd::Zero(3, cps_.size);
        Eigen::MatrixXd g_distance = Eigen::MatrixXd::Zero(3, cps_.size);
        Eigen::MatrixXd g_feasibility = Eigen::MatrixXd::Zero(3, cps_.size);

        calcSmoothnessCost(cps_.points, f_smoothness, g_smoothness);
        calcDistanceCostRebound(cps_.points, f_distance, g_distance, iter_num_, f_smoothness);
        calcFeasibilityCost(cps_.points, f_feasibility, g_feasibility);

        f_combine = lambda1_ * f_smoothness + new_lambda2_ * f_distance + lambda3_ * f_feasibility;

        Eigen::MatrixXd grad_3D = lambda1_ * g_smoothness + new_lambda2_ * g_distance + lambda3_ * g_feasibility;
        memcpy(grad, grad_3D.data() + 3 * order_, n * sizeof(grad[0]));
    }

    void BsplineOptimizer::combineCostRefine(const double *x, double *grad, double &f_combine, const int n)
    {
        memcpy(cps_.points.data() + 3 * order_, x, n * sizeof(x[0]));

        double f_smoothness, f_fitness, f_feasibility;

        Eigen::MatrixXd g_smoothness = Eigen::MatrixXd::Zero(3, cps_.points.cols());
        Eigen::MatrixXd g_fitness = Eigen::MatrixXd::Zero(3, cps_.points.cols());
        Eigen::MatrixXd g_feasibility = Eigen::MatrixXd::Zero(3, cps_.points.cols());

        calcSmoothnessCost(cps_.points, f_smoothness, g_smoothness);
        calcFitnessCost(cps_.points, f_fitness, g_fitness);
        calcFeasibilityCost(cps_.points, f_feasibility, g_feasibility);

        f_combine = lambda1_ * f_smoothness + lambda4_ * f_fitness + lambda3_ * f_feasibility;

        Eigen::MatrixXd grad_3D = lambda1_ * g_smoothness + lambda4_ * g_fitness + lambda3_ * g_feasibility;
        memcpy(grad, grad_3D.data() + 3 * order_, n * sizeof(grad[0]));
    }

} // namespace ego_planner