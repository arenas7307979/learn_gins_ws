#include <iostream>
#include <fstream>
#include <eigen3/Eigen/Dense>
#include <iomanip>
#include "slam_vio/Vsolver/problem.h"
#include "slam_vio/Vsolver/tic_toc.h"
#include "slam_vio/Vsolver/invert4x4_sse.h"

#define DEBUG_LOG_BE 0
#define PRINT_UPDATE_NEAR_ZERO 0
//TODO::
// #define USE_PCG_SOLVER 1
// #ifdef USE_OPENMP
// #include <omp.h>
// #endif

using namespace std;

// define the format you want, you only need one instance of this...
const static Eigen::IOFormat CSVFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", "\n");

void writeToCSVfile(std::string name, Eigen::MatrixXd matrix)
{
    std::ofstream f(name.c_str());
    f << matrix.format(CSVFormat);
}

namespace Vsolver
{
    static inline void invert4x4_to_3x3_SSE(MatXX &src_, MatXX &dst_, int &idx)
    {
        Eigen::Matrix4f src_arg = Eigen::Matrix4f::Identity();
        Eigen::Matrix4f dst_arg = Eigen::Matrix4f::Identity();
        src_arg.topLeftCorner(3, 3) = src_.block(idx, idx, 3, 3).cast<float>();
        invert4x4_SSE(src_arg.data(), dst_arg.data());
        dst_.block(idx, idx, 3, 3) = dst_arg.topLeftCorner(3, 3).cast<double>();
    }

    static inline void invert3x3_HmmVec(float *src, MatXX &hmm_inv, std::vector<int> &pair_idx)
    {
        float det;
        Eigen::Matrix<float, 3, 3,  Eigen::RowMajor> tmp_dst;
        float *tmp_dst_ptr = tmp_dst.data();
        for (int i = 0; i < pair_idx.size(); ++i)
        {
            size_t num_hmm = 9 * i;
            /**
            std::cout << "hmm_inv=" << hmm_inv.block(pair_idx[i], pair_idx[i], 3, 3) <<std::endl;
            std::cout << "src[num_hmm]=" << src[num_hmm] <<std::endl;
            std::cout << "src[num_hmm + 1]=" << src[num_hmm + 1] <<std::endl;
            std::cout << "src[num_hmm + 2]=" << src[num_hmm + 2] <<std::endl;
            std::cout << "src[num_hmm + 3]=" << src[num_hmm + 3] <<std::endl;
            std::cout << "src[num_hmm + 4]=" << src[num_hmm + 4] <<std::endl;
            std::cout << "src[num_hmm + 5]=" << src[num_hmm + 5] <<std::endl;
            std::cout << "src[num_hmm + 6]=" << src[num_hmm + 6] <<std::endl;
            std::cout << "src[num_hmm + 7]=" << src[num_hmm + 7] <<std::endl;
            std::cout << "src[num_hmm + 8]=" << src[num_hmm + 8] <<std::endl;
            **/

            /* Compute adjoint: */
            tmp_dst_ptr[0] = +src[4 + num_hmm] * src[8 + num_hmm] - src[5 + num_hmm] * src[7 + num_hmm];
            tmp_dst_ptr[1] = -src[1 + num_hmm] * src[8 + num_hmm] + src[2 + num_hmm] * src[7 + num_hmm];
            tmp_dst_ptr[2] = +src[1 + num_hmm] * src[5 + num_hmm] - src[2 + num_hmm] * src[4 + num_hmm];
            tmp_dst_ptr[3] = -src[3 + num_hmm] * src[8 + num_hmm] + src[5 + num_hmm] * src[6 + num_hmm];
            tmp_dst_ptr[4] = +src[num_hmm] * src[8 + num_hmm] - src[2 + num_hmm] * src[6 + num_hmm];
            tmp_dst_ptr[5] = -src[num_hmm] * src[5 + num_hmm] + src[2 + num_hmm] * src[3 + num_hmm];
            tmp_dst_ptr[6] = +src[3 + num_hmm] * src[7 + num_hmm] - src[4 + num_hmm] * src[6 + num_hmm];
            tmp_dst_ptr[7] = -src[num_hmm] * src[7 + num_hmm] + src[1 + num_hmm] * src[6 + num_hmm];
            tmp_dst_ptr[8] = +src[num_hmm] * src[4 + num_hmm] - src[1 + num_hmm] * src[3 + num_hmm];

            /* Compute determinant: */
            det = src[num_hmm] * tmp_dst_ptr[0] + src[1 + num_hmm] * tmp_dst_ptr[3] + src[2 + num_hmm] * tmp_dst_ptr[6];

            /* Multiply adjoint with reciprocal of determinant: */
            det = 1.0f / det;

            tmp_dst_ptr[0] *= det;
            tmp_dst_ptr[1] *= det;
            tmp_dst_ptr[2] *= det;
            tmp_dst_ptr[3] *= det;
            tmp_dst_ptr[4] *= det;
            tmp_dst_ptr[5] *= det;
            tmp_dst_ptr[6] *= det;
            tmp_dst_ptr[7] *= det;
            tmp_dst_ptr[8] *= det;
            /*
            std::cout << "tmp_dst[0]=" << tmp_dst.matrix() << std::endl;
            std::cout << "hmm_inv.block(pair_idx[i], pair_idx[i], 3, 3) inv=" << hmm_inv.block(pair_idx[i], pair_idx[i], 3, 3).inverse() << std::endl;
            */
            hmm_inv.block(pair_idx[i], pair_idx[i], 3, 3) = tmp_dst.cast<double>();
        }
    }

    void Problem::LogoutVectorSize()
    {
        // LOG(INFO) <<
        //           "1 problem::LogoutVectorSize verticies_:" << verticies_.size() <<
        //           " edges:" << edges_.size();
    }

    Problem::Problem(ProblemType problemType) : problemType_(problemType)
    {
        LogoutVectorSize();
        verticies_marg_.clear();
    }

    Problem::~Problem()
    {
        //    std::cout << "Problem IS Deleted"<<'\n';
        global_vertex_id = 0;
    }

    bool Problem::AddVertex(std::shared_ptr<Vertex> vertex)
    {
        if (verticies_.find(vertex->Id()) != verticies_.end())
        {
            // LOG(WARNING) << "Vertex " << vertex->Id() << " has been added before";
            return false;
        }
        else
        {
            verticies_.insert(pair<unsigned long, shared_ptr<Vertex>>(vertex->Id(), vertex));
        }
        //TODO::應該沒有意義 因為solveBA最後會重新set prior所有相關參數(Hprior, Jprior...), 拿掉性能變差？
#if 0
    if (problemType_ == ProblemType::SLAM_PROBLEM)
    {
        if (IsPoseVertex(vertex))
        {
            ResizePoseHessiansWhenAddingPose(vertex);
        }
    }
#endif
        return true;
    }

    void Problem::AddOrderingSLAM(std::shared_ptr<Vsolver::Vertex> v)
    {
        if (IsPoseVertex(v))
        {
            v->SetOrderingId(ordering_poses_);
            idx_pose_vertices_.insert(pair<ulong, std::shared_ptr<Vertex>>(v->Id(), v));
            ordering_poses_ += v->LocalDimension();
        }
        else if (IsLandmarkVertex(v))
        {
            v->SetOrderingId(ordering_landmarks_);
            ordering_landmarks_ += v->LocalDimension();
            idx_landmark_vertices_.insert(pair<ulong, std::shared_ptr<Vertex>>(v->Id(), v));
        }
    }

    void Problem::ResizePoseHessiansWhenAddingPose(shared_ptr<Vertex> v)
    {
        //TODO:: 應該沒有意義 因為solveBA最後會重新set prior所有相關參數(Hprior, Jprior...)
        int size = H_prior_.rows() + v->LocalDimension();
        H_prior_.conservativeResize(size, size);
        b_prior_.conservativeResize(size);

        b_prior_.tail(v->LocalDimension()).setZero();
        H_prior_.rightCols(v->LocalDimension()).setZero();
        H_prior_.bottomRows(v->LocalDimension()).setZero();
    }
    void Problem::ExtendHessiansPriorSize(int dim)
    {
        int size = H_prior_.rows() + dim;
        H_prior_.conservativeResize(size, size);
        b_prior_.conservativeResize(size);

        b_prior_.tail(dim).setZero();
        H_prior_.rightCols(dim).setZero();
        H_prior_.bottomRows(dim).setZero();
    }

    bool Problem::IsPoseVertex(std::shared_ptr<Vsolver::Vertex> v)
    {
        string type = v->TypeInfo();
        return type == string("VertexPose") ||
               type == string("VertexSpeed") ||
               type == string("VertexBias") ||
               type == string("VertexSO3") ||
               type == string("VertexCamPara") ||
               type == string("VertexTd")||
               type == string("VertexTranslation") ||
               type == string("VertexSpeedBias") ||
               type == string("VertexPose_No_Tz");
    }

    bool Problem::IsLandmarkVertex(std::shared_ptr<Vsolver::Vertex> v)
    {
        string type = v->TypeInfo();
        return type == string("VertexPointXYZ") ||
               type == string("VertexInverseDepth");
    }

    bool Problem::AddEdge(shared_ptr<Edge> edge)
    {
        if (edges_.find(edge->Id()) == edges_.end())
        {
            edges_.insert(pair<ulong, std::shared_ptr<Edge>>(edge->Id(), edge));
        }
        else
        {
            // LOG(WARNING) << "Edge " << edge->Id() << " has been added before!";
            return false;
        }

        for (auto &vertex : edge->Verticies())
        {
            vertexToEdge_.insert(pair<ulong, shared_ptr<Edge>>(vertex->Id(), edge));
        }
        return true;
    }

    vector<shared_ptr<Edge>> Problem::GetConnectedEdges(std::shared_ptr<Vertex> vertex)
    {
        vector<shared_ptr<Edge>> edges;
        auto range = vertexToEdge_.equal_range(vertex->Id());
        for (auto iter = range.first; iter != range.second; ++iter)
        {

            // 并且这个edge还需要存在，而不是已经被remove了
            if (edges_.find(iter->second->Id()) == edges_.end())
                continue;

            edges.emplace_back(iter->second);
        }
        return edges;
    }

    bool Problem::RemoveVertex(std::shared_ptr<Vertex> vertex)
    {
        //check if the vertex is in map_verticies_
        if (verticies_.find(vertex->Id()) == verticies_.end())
        {
            // LOG(WARNING) << "The vertex " << vertex->Id() << " is not in the problem!" << endl;
            return false;
        }

        // 这里要 remove 该顶点对应的 edge.
        vector<shared_ptr<Edge>> remove_edges = GetConnectedEdges(vertex);
        for (size_t i = 0; i < remove_edges.size(); i++)
        {
            RemoveEdge(remove_edges[i]);
        }

        if (IsPoseVertex(vertex))
            idx_pose_vertices_.erase(vertex->Id());
        else
            idx_landmark_vertices_.erase(vertex->Id());

        vertex->SetOrderingId(-1); // used to debug
        verticies_.erase(vertex->Id());
        vertexToEdge_.erase(vertex->Id());

        return true;
    }

    bool Problem::RemoveEdge(std::shared_ptr<Edge> edge)
    {
        //check if the edge is in map_edges_
        if (edges_.find(edge->Id()) == edges_.end())
        {
            // LOG(WARNING) << "The edge " << edge->Id() << " is not in the problem!" << endl;
            return false;
        }

        edges_.erase(edge->Id());
        return true;
    }

    bool Problem::Solve(int iterations)
    {
        if (edges_.size() == 0 || verticies_.size() == 0)
        {
            std::cerr << "\nCannot solve problem without edges or verticies" << '\n';
            return false;
        }

        TicToc t_solve;
        // 统计优化变量的维数，为构建 H 矩阵做准备
        SetOrdering();
        // 遍历edge, 构建 H 矩阵
#ifdef USE_OPENMP
        MakeHessianOmp();
#else
        MakeHessian();
#endif

        //first iteration : remove Hessian_ eigenvalue when it value too small
        // double eps = 1e-8;
        // Eigen::MatrixXd Hessian_tmp = 0.5 * (Hessian_ + Hessian_.transpose());
        // Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> saes(Hessian_tmp);
        // Hessian_ = saes.eigenvectors() * Eigen::VectorXd((saes.eigenvalues().array() > eps).select(saes.eigenvalues().array(), 0)).asDiagonal() *
        //          saes.eigenvectors().transpose();

        // LM 初始化
        ComputeLambdaInitLM();
        // LM 算法迭代求解
        bool stop = false;
        int iter = 0;
        double last_chi_ = 1e20;
        while (!stop && (iter < iterations))
        {
#if DEBUG_LOG_BE
            std::cout << "iter: " << iter << " , chi= " << currentChi_ << " , Lambda= " << currentLambda_ << '\n';
#endif
            bool oneStepSuccess = false;
            int false_cnt = 0;
            while (!oneStepSuccess && false_cnt < 10) // 不断尝试 Lambda, 直到成功迭代一步
            {
                // setLambda
                //            AddLambdatoHessianLM();
                // 第四步，解线性方程
                SolveLinearSystem();
                //
                //            RemoveLambdaHessianLM();

                // 优化退出条件1： delta_x_ 很小则退出
                //            if (delta_x_.squaredNorm() <= 1e-6 || false_cnt > 10)
                // TODO:: 退出条件还是有问题, 好多次误差都没变化了，还在迭代计算，应该搞一个误差不变了就中止
                //            if ( false_cnt > 10)
                //            {
                //                stop = true;
                //                break;
                //            }

                // 更新状态量
                UpdateStates();
                // 判断当前步是否可行以及 LM 的 lambda 怎么更新, chi2 也计算一下
                oneStepSuccess = IsGoodStepInLM();
                // 后续处理，
                if (oneStepSuccess)
                {
                    //                std::cout << " get one step success\n";

                    // 在新线性化点 构建 hessian
#ifdef USE_OPENMP
                    MakeHessianOmp();
#else
                    MakeHessian();
#endif
                    // TODO:: 这个判断条件可以丢掉，条件 b_max <= 1e-12 很难达到，这里的阈值条件不应该用绝对值，而是相对值
                    //                double b_max = 0.0;
                    //                for (int i = 0; i < b_.size(); ++i) {
                    //                    b_max = max(fabs(b_(i)), b_max);
                    //                }
                    //                // 优化退出条件2： 如果残差 b_max 已经很小了，那就退出
                    //                stop = (b_max <= 1e-12);
                    false_cnt = 0;
                }
                else
                {
                    false_cnt++;
                    RollbackStates(); // 误差没下降，回滚
                }
            }
            iter++;

            // 优化退出条件3： currentChi_ 跟第一次的 chi2 相比，下降了 1e6 倍则退出
            // TODO:: 应该改成前后两次的误差已经不再变化
            //        if (sqrt(currentChi_) <= stopThresholdLM_)
            //        if (sqrt(currentChi_) < 1e-15)
            if (last_chi_ - currentChi_ < 1e-5)
            {
#if DEBUG_LOG_
                std::cout << "sqrt(currentChi_) <= stopThresholdLM_" << '\n';
#endif
                stop = true;
            }
            last_chi_ = currentChi_;
        }
        //std::cout << "problem solve cost: " << t_solve.toc() << " ms" << '\n';
        //std::cout << "   makeHessian cost: " << t_hessian_cost_ << " ms" << '\n';
        static long iter_sum = 0;
        static double t_solve_cost_sum = 0;
        static double t_hessian_cost_sum = 0;
        static int cnt = 0;
        iter_sum += iter;
        t_solve_cost_sum += t_solve.toc();
        t_hessian_cost_sum += t_hessian_cost_;
        cnt++;
#if DEBUG_LOG_BE
        std::cout << "avg iter: " << (double)iter_sum / (double)cnt << '\n';
        std::cout << "avg problem solve cost: " << t_solve_cost_sum / (double)cnt << " ms" << '\n';
        std::cout << "avg makeHessian cost: " << t_hessian_cost_sum / (double)cnt << " ms" << '\n';
#endif
        t_hessian_cost_ = 0.;
        return true;
    }

    bool Problem::SolveWithDogLeg(int iterations)
    {
        if (edges_.size() == 0 || verticies_.size() == 0)
        {
            std::cerr << "\nCannot solve problem without edges or verticies" << '\n';
            return false;
        }
        TicToc t_solve;
        // 统计优化变量的维数，为构建 H 矩阵做准备
        SetOrdering();
        // 遍历edge, 构建 H 矩阵
#ifdef USE_OPENMP
        MakeHessianOmp();
#else
        MakeHessian();
#endif

        //first iteration : remove Hessian_ eigenvalue when it value too small
        //double eps = 1e-12;
        //Eigen::MatrixXd Hessian_tmp = 0.5 * (Hessian_ + Hessian_.transpose());
        //Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> saes(Hessian_tmp);
        //Hessian_ = saes.eigenvectors() * Eigen::VectorXd((saes.eigenvalues().array() > eps).select(saes.eigenvalues().array(), 0)).asDiagonal() *
        //           saes.eigenvectors().transpose();

        InitDL();
        bool stop = false;
        int iter = 0;
        double last_chi_ = 1e20;
        stop = currentChi_ < DL_epsilon_3_ || b_.norm() < DL_epsilon_1_;
        while (!stop && (iter < iterations))
        {
#if DEBUG_LOG_BE
            std::cout << "iter: " << iter << " , chi= " << currentChi_ << " , Delta= " << DL_delta_ << '\n';
#endif
            bool oneStepSuccess = false;
            int false_cnt = 0;
            while (!oneStepSuccess && false_cnt < 5) // 不断尝试 delta, 直到成功迭代一步
            {
                SolveLinearSystem();
                //calculate delta_x_
                computeDogLegUpdate();
                VecX trial_param(VecX::Zero(ordering_generic_));
                for (auto vertex : verticies_)
                {
                    ulong idx = vertex.second->OrderingId();
                    ulong dim = vertex.second->LocalDimension();
                    VecX vertex_param, vertex_backup_param;
                    vertex_param = vertex.second->Parameters();
                    for (size_t i = 0; i < dim; ++i)
                    {
                        trial_param(i + idx) = vertex_param(i);
                    }
                }
                if (delta_x_.norm() <= DL_epsilon_2_ * (trial_param.norm() + DL_epsilon_2_))
                {
                    stop = true;
                    #if PRINT_UPDATE_NEAR_ZERO
                    printf("optimize exit because DL_delta_4 update is near zero.\n");
                    #endif
                    break;
                }
                UpdateStates();
                oneStepSuccess = IsGoodStepInDL();
                if (oneStepSuccess)
                {
#ifdef USE_OPENMP
                    MakeHessianOmp();
#else
                    MakeHessian();
#endif
                    false_cnt = 0;
                }
                else
                {
                    false_cnt++;
                    RollbackStates(); // 误差没下降，回滚
                }
            }
            iter++;
            if (last_chi_ - currentChi_ < 1e-5)
            {
#if DEBUG_LOG_
                std::cout << "sqrt(currentChi_) <= stopThresholdLM_" << '\n';
#endif
                stop = true;
            }
            last_chi_ = currentChi_;
            if (currentChi_ <= DL_epsilon_3_)
            {
                stop = true;
                #if PRINT_UPDATE_NEAR_ZERO
                printf("optimize exit because DL_epsilon_3_ error is near zero.\n");
                #endif
                continue;
            }
            if (b_.norm() <= DL_epsilon_1_)
            {
                stop = true;
                #if PRINT_UPDATE_NEAR_ZERO
                printf("optimize exit because DL_epsilon_1_ Jt_res is near zero.\n");
                #endif
                continue;
            }
            VecX trial_param(VecX::Zero(ordering_generic_));
            for (auto vertex : verticies_)
            {
                ulong idx = vertex.second->OrderingId();
                ulong dim = vertex.second->LocalDimension();
                VecX vertex_param, vertex_backup_param;
                vertex_param = vertex.second->Parameters();
                for (size_t i = 0; i < dim; ++i)
                {
                    trial_param(i + idx) = vertex_param(i);
                }
            }
            if (DL_delta_ <= DL_epsilon_2_ * (trial_param.norm() + DL_epsilon_2_))
            {
                stop = true;
                #if PRINT_UPDATE_NEAR_ZERO
                printf("optimize exit because DL_delta_2 dog-leg delta is near zero.\n");
                #endif
                continue;
            }
        }
        static long iter_sum = 0;
        static double t_solve_cost_sum = 0;
        static double t_hessian_cost_sum = 0;
        static int cnt = 0;
        iter_sum += iter;
        t_solve_cost_sum += t_solve.toc();
        t_hessian_cost_sum += t_hessian_cost_;
        cnt++;
#if DEBUG_LOG_BE
        std::cout << "avg iter: " << (double)iter_sum / (double)cnt << '\n';
        std::cout << "avg problem solve cost: " << t_solve_cost_sum / (double)cnt << " ms" << '\n';
        std::cout << "avg makeHessian cost: " << t_hessian_cost_sum / (double)cnt << " ms" << '\n';
#endif
        t_hessian_cost_ = 0.;

        return true;
    }

    void Problem::InitDL()
    {
        DL_delta_ = 1.;
        DL_epsilon_1_ = 1e-12;
        DL_epsilon_2_ = 1e-12;
        DL_epsilon_3_ = 1e-12;
        currentLambda_ = -1;
        currentChi_ = 0.0;

        for (auto edge : edges_)
        {
            currentChi_ += edge.second->RobustChi2();
        }
        if (err_prior_.rows() > 0)
            currentChi_ += err_prior_.norm();
        currentChi_ *= 0.5;
    }

    void Problem::computeDogLegUpdate()
    {
        VecX update_gn = delta_x_;
        auto alpha_matrix = (b_.transpose() * b_) / (b_.transpose() * Hessian_ * b_);
        double alpha_ = alpha_matrix(0, 0);
        double b_norm = b_.norm();
        //VecX update_sd = alpha_ * b_;
        if (update_gn.norm() <= DL_delta_)
        {
            return;
        }
        else if (alpha_ * b_norm >= DL_delta_)
        {
            delta_x_ = (DL_delta_ / b_norm) * b_;
        }
        else
        {
            VecX j = alpha_ * b_;
            const VecX &k = update_gn;
            double c = j.transpose() * (k - j);
            double DL_beta = 0.;
            double k_mins_j_squaredNorm = (k - j).squaredNorm();
            //equation (3.20b)
            if (c <= 0)
            {
                DL_beta = (-c + sqrt(c * c + k_mins_j_squaredNorm * (pow(DL_delta_, 2) - pow(alpha_ * b_norm, 2)))) / k_mins_j_squaredNorm;
            }
            else
            {
                DL_beta = (pow(DL_delta_, 2) - pow(alpha_ * b_norm, 2)) / (c + sqrt(c * c + k_mins_j_squaredNorm * (pow(DL_delta_, 2) - pow(alpha_ * b_norm, 2))));
            }
            delta_x_ = j + DL_beta * (k - j);
        }
        return;
    }

    bool Problem::IsGoodStepInDL()
    {
        double scale = 0;
#if 1
        scale = 0.5 * delta_x_.transpose() * b_;
        scale += 1e-6; // make sure it's non-zero :)
#else
        scale = delta_x_.transpose() * b_;
        scale -= 0.5 * delta_x_.transpose() * Hessian_ * delta_x_;
        scale += 1e-6; // make sure it's non-zero :)
#endif

        // recompute residuals after update state
        double tempChi = 0.0;
        for (auto edge : edges_)
        {
            edge.second->ComputeResidual();
            tempChi += edge.second->RobustChi2();
        }
        if (err_prior_.size() > 0)
            tempChi += err_prior_.norm();
        tempChi *= 0.5; // 1/2 * err^2
        if (!isfinite(tempChi))
        {
            return false;
        }
        double rho = (currentChi_ - tempChi) / scale;

#if 0
    if (rho > 0.75) {
        DL_delta_ = std::max(DL_delta_, 3 * delta_x_.norm());
    }
    else if (rho < 0.25) {
        DL_delta_ /= 2.0;
    }
    if (rho > 0 && isfinite(tempChi))  {
        currentChi_ = tempChi;
        vsolver::Trace::TraceEnd();
        return true;
    }
    else {
        vsolver::Trace::TraceEnd();
        return false;
    }
#else
        //迭代策略
        if (rho > 0 && isfinite(tempChi))
        {
            if (rho > 0.75)
                DL_delta_ = std::max(DL_delta_, 3 * delta_x_.norm());
            else if (rho < 0.25)
                DL_delta_ /= 2.0;

            currentChi_ = tempChi;
            return true;
        }
        else
        {
            DL_delta_ /= 2.0;
            return false;
        }
#endif
    }

    bool Problem::SolveGenericProblem(int iterations)
    {
        return true;
    }

    void Problem::SetOrdering()
    {

        // 每次重新计数
        ordering_poses_ = 0;
        ordering_generic_ = 0;
        ordering_landmarks_ = 0;

        // Note:: verticies_ 是 map 类型的, 顺序是按照 id 号排序的
        for (auto vertex : verticies_)
        {
            ordering_generic_ += vertex.second->LocalDimension(); // 所有的优化变量总维数

            if (problemType_ == ProblemType::SLAM_PROBLEM) // 如果是 slam 问题，还要分别统计 pose 和 landmark 的维数，后面会对他们进行排序
            {
                AddOrderingSLAM(vertex.second);
            }
        }

        if (problemType_ == ProblemType::SLAM_PROBLEM)
        {
            // 这里要把 landmark 的 ordering 加上 pose 的数量，就保持了 landmark 在后,而 pose 在前
            ulong all_pose_dimension = ordering_poses_;
            for (auto landmarkVertex : idx_landmark_vertices_)
            {
                landmarkVertex.second->SetOrderingId(
                    landmarkVertex.second->OrderingId() + all_pose_dimension);
            }
        }

        //    CHECK_EQ(CheckOrdering(), true);
    }

    bool Problem::CheckOrdering()
    {
        if (problemType_ == ProblemType::SLAM_PROBLEM)
        {
            int current_ordering = 0;
            for (auto v : idx_pose_vertices_)
            {
                assert(v.second->OrderingId() == current_ordering);
                current_ordering += v.second->LocalDimension();
            }

            for (auto v : idx_landmark_vertices_)
            {
                assert(v.second->OrderingId() == current_ordering);
                current_ordering += v.second->LocalDimension();
            }
        }
        return true;
    }

    void Problem::MakeHessian()
    {
        TicToc t_h;
        // 直接构造大的 H 矩阵
        ulong size = ordering_generic_;
        MatXX H(MatXX::Zero(size, size));
        VecX b(VecX::Zero(size));

        for (auto &edge : edges_)
        {
            edge.second->ComputeResidual();
            edge.second->ComputeJacobians();

            // TODO:: robust cost
            auto jacobians = edge.second->Jacobians();
            auto verticies = edge.second->Verticies();
            assert(jacobians.size() == verticies.size());
            for (size_t i = 0; i < verticies.size(); ++i)
            {
                auto v_i = verticies[i];
                if (v_i->IsFixed())
                    continue; // Hessian 里不需要添加它的信息，也就是它的雅克比为 0

                auto jacobian_i = jacobians[i];

                ulong index_i = v_i->OrderingId();
                ulong dim_i = v_i->LocalDimension();

                // 鲁棒核函数会修改残差和信息矩阵，如果没有设置 robust cost function，就会返回原来的
                double drho;
                MatXX robustInfo(edge.second->Information().rows(), edge.second->Information().cols());
                edge.second->RobustInfo(drho, robustInfo);

                MatXX JtW = jacobian_i.transpose() * robustInfo;
                for (size_t j = i; j < verticies.size(); ++j)
                {
                    auto v_j = verticies[j];

                    if (v_j->IsFixed())
                        continue;

                    auto jacobian_j = jacobians[j];
                    ulong index_j = v_j->OrderingId();
                    ulong dim_j = v_j->LocalDimension();

                    assert(v_j->OrderingId() != -1);
                    MatXX hessian = JtW * jacobian_j;

                    // 所有的信息矩阵叠加起来
                    H.block(index_i, index_j, dim_i, dim_j).noalias() += hessian;
                    if (j != i)
                    {
                        // 对称的下三角
                        // TODO:: By filling the numbers back in through the symmetric matrix, we can reduce the addition operation
                        H.block(index_j, index_i, dim_j, dim_i).noalias() += hessian.transpose();
                    }
                }
                b.segment(index_i, dim_i).noalias() -= drho * jacobian_i.transpose() * edge.second->Information() * edge.second->Residual();
            }
        }
        Hessian_ = H;
        b_ = b;
        t_hessian_cost_ += t_h.toc();

        //TODO:: Add loss function like loss_function from marginalization_factor.cpp
        if (H_prior_.rows() > 0)
        {
            MatXX H_prior_tmp = H_prior_;
            VecX b_prior_tmp = b_prior_;

            //*NOTE 遍历所有 POSE 顶点，然后设置相应的先验维度为 0 .  fix 外参数, SET PRIOR TO ZERO
            //*     這邊主要功能是探索有無vertex啟動了fixed的功能, 當有節點使用鎖, 
            //*     即使他在prior項有相關我們也必須設定為0[set prior H and b to zero], 由於先驗跟landmark無關, 因此無須遍歷landmark
            for (auto vertex : verticies_)
            {
                if (IsPoseVertex(vertex.second) && vertex.second->IsFixed())
                {
                    int idx = vertex.second->OrderingId();
                    int dim = vertex.second->LocalDimension();
                    H_prior_tmp.block(idx, 0, dim, H_prior_tmp.cols()).setZero();
                    H_prior_tmp.block(0, idx, H_prior_tmp.rows(), dim).setZero();
                    b_prior_tmp.segment(idx, dim).setZero();
                    //std::cout << " fixed prior, set the Hprior and bprior part to zero, idx: "<<idx <<" dim: "<<dim<<'\n';
                }
            }
            Hessian_.topLeftCorner(ordering_poses_, ordering_poses_) += H_prior_tmp;
            b_.head(ordering_poses_) += b_prior_tmp;
        }
        delta_x_ = VecX::Zero(size); // initial delta_x = 0_n;
    }

// TODO:: accelate
#ifdef USE_OPENMP
    void Problem::MakeHessianOmp()
    {
        TicToc t_h;
        const int kMaxThreadNum = 16;
        int thread_num = std::min(omp_get_max_threads(), kMaxThreadNum);
        // 直接构造大的 H 矩阵
        ulong size = ordering_generic_;
        vector<MatXX> H(thread_num, MatXX::Zero(size, size));
        vector<VecX> b(thread_num, VecX::Zero(size));
        Hessian_ = MatXX::Zero(size, size);
        b_ = VecX::Zero(size);
        // TODO:: accelate
        vector<unsigned long> edge_ids;
        for (const auto &edge : edges_)
        {
            edge_ids.emplace_back(edge.first);
        }
        /**#pragma omp parallel for是OpenMP中的一个指令，表示接下来的for循环将被多线程执行，另外每次循环之间不能有关系 **/
#pragma omp parallel for num_threads(thread_num)
        for (size_t edge_index = 0; edge_index < edge_ids.size(); ++edge_index)
        {
            auto &edge = edges_[edge_ids[edge_index]];
            edge->ComputeResidual();
            edge->ComputeJacobians();

            // TODO:: robust cost
            auto jacobians = edge->Jacobians();
            auto verticies = edge->Verticies();
            assert(jacobians.size() == verticies.size());
            int tid = omp_get_thread_num();
            for (size_t i = 0; i < verticies.size(); ++i)
            {
                auto v_i = verticies[i];
                if (v_i->IsFixed())
                    continue; // Hessian 里不需要添加它的信息，也就是它的雅克比为 0

                auto jacobian_i = jacobians[i];
                ulong index_i = v_i->OrderingId();
                ulong dim_i = v_i->LocalDimension();

                // 鲁棒核函数会修改残差和信息矩阵，如果没有设置 robust cost function，就会返回原来的
                double drho;
                MatXX robustInfo(edge->Information().rows(), edge->Information().cols());
                edge->RobustInfo(drho, robustInfo);
                MatXX JtW = jacobian_i.transpose() * robustInfo;
                for (size_t j = i; j < verticies.size(); ++j)
                {
                    auto v_j = verticies[j];

                    if (v_j->IsFixed())
                        continue;

                    auto jacobian_j = jacobians[j];
                    ulong index_j = v_j->OrderingId();
                    ulong dim_j = v_j->LocalDimension();

                    assert(v_j->OrderingId() != -1);
                    MatXX hessian = JtW * jacobian_j;
                    // 所有的信息矩阵叠加起来
                    H[tid].block(index_i, index_j, dim_i, dim_j).noalias() += hessian;
                    if (j != i)
                    {
                        // 对称的下三角
                        H[tid].block(index_j, index_i, dim_j, dim_i).noalias() += hessian.transpose();
                    }
                }
                b[tid].segment(index_i, dim_i).noalias() -= drho * jacobian_i.transpose() * edge->Information() * edge->Residual();
            }
        }

        for (size_t i = 0; i < thread_num; ++i)
        {
            Hessian_ += H[i];
            b_ += b[i];
        }
        t_hessian_cost_ += t_h.toc();

        if (H_prior_.rows() > 0)
        {
            MatXX H_prior_tmp = H_prior_;
            VecX b_prior_tmp = b_prior_;

            /// 遍历所有 POSE 顶点，然后设置相应的先验维度为 0 .  fix 外参数, SET PRIOR TO ZERO
            /// landmark 没有先验
            for (auto vertex : verticies_)
            {
                if (IsPoseVertex(vertex.second) && vertex.second->IsFixed())
                {
                    int idx = vertex.second->OrderingId();
                    int dim = vertex.second->LocalDimension();
                    H_prior_tmp.block(idx, 0, dim, H_prior_tmp.cols()).setZero();
                    H_prior_tmp.block(0, idx, H_prior_tmp.rows(), dim).setZero();
                    b_prior_tmp.segment(idx, dim).setZero();
                    //std::cout << " fixed prior, set the Hprior and bprior part to zero, idx: "<<idx <<" dim: "<<dim<<'\n';
                }
            }
            Hessian_.topLeftCorner(ordering_poses_, ordering_poses_) += H_prior_tmp;
            b_.head(ordering_poses_) += b_prior_tmp;
        }

        delta_x_ = VecX::Zero(size); // initial delta_x = 0_n;
    }
#endif

    /*
 * Solve Hx = b, we can use PCG iterative method or use sparse Cholesky
 */
    void Problem::SolveLinearSystem()
    {
        if (problemType_ == ProblemType::GENERIC_PROBLEM)
        {
            // PCG solver
            MatXX H = Hessian_;
            if (currentLambda_ > 0)
            {
                for (size_t i = 0; i < Hessian_.cols(); ++i)
                {
                    H(i, i) += currentLambda_;
                }
            }
            // delta_x_ = PCGSolver(H, b_, H.rows() * 2);
            delta_x_ = H.ldlt().solve(b_);
        }
        else
        {
            //TicToc t_Hmminv;
            // step1: schur marginalization --> Hpp, bpp
            int reserve_size = ordering_poses_;
            int marg_size = ordering_landmarks_;
            MatXX Hmm = Hessian_.block(reserve_size, reserve_size, marg_size, marg_size);
            MatXX Hpm = Hessian_.block(0, reserve_size, reserve_size, marg_size);
            MatXX Hmp = Hessian_.block(reserve_size, 0, marg_size, reserve_size);
            VecX bpp = b_.segment(0, reserve_size);
            VecX bmm = b_.segment(reserve_size, marg_size);

            if (marg_size != 0)
            {
                // Hmm 是对角线矩阵，它的求逆可以直接为对角线块分别求逆，如果是逆深度，对角线块为1维的，则直接为对角线的倒数，这里可以加速
                MatXX Hmm_inv(MatXX::Zero(marg_size, marg_size));
                for (auto landmarkVertex : idx_landmark_vertices_)
                {
                    int idx = landmarkVertex.second->OrderingId() - reserve_size;
                    int size = landmarkVertex.second->LocalDimension();
                    if (size == 1)
                    {
                        //**NOTE:: one-dim [inverse_depth] Hmm_inv and Hmm 1*1 doule number, size = 1
                        Hmm_inv.block(idx, idx, size, size)(0, 0) = (1.0 / Hmm.block(idx, idx, size, size)(0, 0));
                    }
                    else if (size == 3)
                    {
                        //TODO:: [Important] Accelerate the diagonal element inverse
                        //[ref : 4bc530a0d]
                        Hmm_inv.block(idx, idx, size, size) = Hmm.block(idx, idx, size, size).inverse();
                    }
                }
                MatXX tempH = Hpm * Hmm_inv;
                H_pp_schur_ = Hessian_.block(0, 0, ordering_poses_, ordering_poses_) - tempH * Hmp;
                b_pp_schur_ = bpp - tempH * bmm;
                // step2: solve Hpp * delta_x = bpp
                VecX delta_x_pp(VecX::Zero(reserve_size));
                if (currentLambda_ > 0)
                {
                    for (ulong i = 0; i < ordering_poses_; ++i)
                    {
                        H_pp_schur_(i, i) += currentLambda_; // LM Method
                    }
                }

                // TicToc t_linearsolver;
                //TODO:: PCG solver
                /*
                if (H_pp_schur_.rows() == H_pp_schur_.cols())
                {
                    vsolver::Trace::TraceBegin("PCGSolver");
                    delta_x_ = PCGSolver(H_pp_schur_, b_pp_schur_, H_pp_schur_.rows() * 2);
                    vsolver::Trace::TraceEnd();
                }
                */
                {
                    delta_x_pp = H_pp_schur_.ldlt().solve(b_pp_schur_); //  SVec.asDiagonal() * svd.matrixV() * Ub;
                }
                delta_x_.head(reserve_size) = delta_x_pp;

                // step3: solve Hmm * delta_x = bmm - Hmp * delta_x_pp;
                VecX delta_x_ll(marg_size);
                delta_x_ll = Hmm_inv * (bmm - Hmp * delta_x_pp);
                delta_x_.tail(marg_size) = delta_x_ll;
            }
            else
            {
                H_pp_schur_ = Hessian_.block(0, 0, ordering_poses_, ordering_poses_);
                b_pp_schur_ = bpp;
                // step2: solve Hpp * delta_x = bpp
                VecX delta_x_pp(VecX::Zero(reserve_size));
                if (currentLambda_ > 0)
                {
                    for (ulong i = 0; i < ordering_poses_; ++i)
                    {
                        H_pp_schur_(i, i) += currentLambda_; // LM Method
                    }
                }

                // TicToc t_linearsolver;
                //TODO:: PCG solver
                /*
                if (H_pp_schur_.rows() == H_pp_schur_.cols())
                {
                    vsolver::Trace::TraceBegin("PCGSolver");
                    delta_x_ = PCGSolver(H_pp_schur_, b_pp_schur_, H_pp_schur_.rows() * 2);
                    vsolver::Trace::TraceEnd();
                }
                */

                delta_x_pp = H_pp_schur_.ldlt().solve(b_pp_schur_); //  SVec.asDiagonal() * svd.matrixV() * Ub;
                delta_x_.head(reserve_size) = delta_x_pp;
            }
        }
    }

    void Problem::UpdateStates()
    {
        // update vertex
        for (auto vertex : verticies_)
        {
            vertex.second->BackUpParameters(); // 保存上次的估计值

            ulong idx = vertex.second->OrderingId();
            ulong dim = vertex.second->LocalDimension();
            VecX delta = delta_x_.segment(idx, dim);
            vertex.second->Plus(delta);
        }

        // update prior
        if (err_prior_.rows() > 0)
        {
            // BACK UP b_prior_
            b_prior_backup_ = b_prior_;
            err_prior_backup_ = err_prior_;

            /// update with first order Taylor, b' = b + \frac{\delta b}{\delta x} * \delta x
            /// delta x = Computes the linearized deviation from the references (linearization points)
            b_prior_ -= H_prior_ * delta_x_.head(ordering_poses_); // update the error_prior
            err_prior_ = -Jt_prior_inv_ * b_prior_.head(ordering_poses_ - 15);

            // std::cout << "                : "<< b_prior_.norm()<<" " <<err_prior_.norm()<< '\n';
            // std::cout << "     delta_x_ ex: "<< delta_x_.head(6).norm() << '\n';
        }
    }

    void Problem::RollbackStates()
    {

        // update vertex
        for (auto vertex : verticies_)
        {
            vertex.second->RollBackParameters();
        }

        // Roll back prior_
        if (err_prior_.rows() > 0)
        {
            b_prior_ = b_prior_backup_;
            err_prior_ = err_prior_backup_;
        }
    }

    /// LM
    void Problem::ComputeLambdaInitLM()
    {
        ni_ = 2.;
        currentLambda_ = -1.;
        currentChi_ = 0.0;

        for (auto edge : edges_)
        {
            currentChi_ += edge.second->RobustChi2();
        }
        if (err_prior_.rows() > 0)
            currentChi_ += err_prior_.norm();
        currentChi_ *= 0.5;

        stopThresholdLM_ = 1e-10 * currentChi_; // 迭代条件为 误差下降 1e-6 倍

        double maxDiagonal = 0;
        ulong size = Hessian_.cols();
        assert(Hessian_.rows() == Hessian_.cols() && "Hessian is not square");
        for (ulong i = 0; i < size; ++i)
        {
            maxDiagonal = std::max(fabs(Hessian_(i, i)), maxDiagonal);
        }

        maxDiagonal = std::min(5e10, maxDiagonal);
        double tau = 1e-5; // 1e-5
        currentLambda_ = tau * maxDiagonal;
        //        std::cout << "currentLamba_: "<<maxDiagonal<<" "<<currentLambda_<<'\n';
    }

    void Problem::AddLambdatoHessianLM()
    {
        ulong size = Hessian_.cols();
        assert(Hessian_.rows() == Hessian_.cols() && "Hessian is not square");
        for (ulong i = 0; i < size; ++i)
        {
            Hessian_(i, i) += currentLambda_;
        }
    }

    void Problem::RemoveLambdaHessianLM()
    {
        ulong size = Hessian_.cols();
        assert(Hessian_.rows() == Hessian_.cols() && "Hessian is not square");
        // TODO:: 这里不应该减去一个，数值的反复加减容易造成数值精度出问题？而应该保存叠加lambda前的值，在这里直接赋值
        for (ulong i = 0; i < size; ++i)
        {
            Hessian_(i, i) -= currentLambda_;
        }
    }

    bool Problem::IsGoodStepInLM()
    {

        // recompute residuals after update state
        double tempChi = 0.0;
        for (auto edge : edges_)
        {
            edge.second->ComputeResidual();
            tempChi += edge.second->RobustChi2();
        }
        if (err_prior_.size() > 0)
            tempChi += err_prior_.norm();
        tempChi *= 0.5; // 1/2 * err^2

        //     double rho = (currentChi_ - tempChi) / scale;
        //     if (rho > 0 && isfinite(tempChi))   // last step was good, 误差在下降
        //     {
        //         double alpha = 1. - pow((2 * rho - 1), 3);
        //         alpha = std::min(alpha, 2. / 3.);
        //         double scaleFactor = (std::max)(1. / 3., alpha);
        //         currentLambda_ *= scaleFactor;
        //         ni_ = 2;
        //         currentChi_ = tempChi;
        //         return true;
        //     } else {
        //         currentLambda_ *= ni_;
        //         ni_ *= 2;
        //         return false;
        //     }
        static const double beta = 0.02;
        if (isfinite(tempChi) && tempChi < currentChi_)
        {
            ulong size = ordering_generic_;
            VecX trial_param(VecX::Zero(size));
            VecX backup_param(VecX::Zero(size));
            for (auto vertex : verticies_)
            {
                ulong idx = vertex.second->OrderingId();
                ulong dim = vertex.second->LocalDimension();
                VecX vertex_param, vertex_backup_param;
                vertex_param = vertex.second->Parameters();
                vertex_backup_param = vertex.second->getBackupParameters();
                for (size_t i = 0; i < dim; ++i)
                {
                    trial_param(i + idx) = vertex_param(i);
                    backup_param(i + idx) = vertex_backup_param(i);
                }
            }
            double wip = trial_param.dot(backup_param) / trial_param.norm() / backup_param.norm();
            if (!isnan(wip))
            {
                currentLambda_ = currentLambda_ * pow(beta, wip);
            }
            currentChi_ = tempChi;
            return true;
        }
        else
        {
            //        Eigen::JacobiSVD<Eigen::MatrixXd> svd(Hessian_, Eigen::ComputeThinU|Eigen::ComputeThinV);
            //        if(svd.singularValues()(svd.singularValues().size()-1) > 0)
            //            currentLambda_ = currentLambda_ / beta;
            //        else{
            //            vector<double> dii(Hessian_.rows(), 0);
            //            double min_dii = INT_MAX;
            //            for(int i=0; i<Hessian_.rows();i++){
            //                for(int j=0;j<Hessian_.cols();j++){
            //                    if(j!=i)
            //                        dii[i] += Hessian_(i,j);
            //                }
            //                min_dii = min(min_dii, dii[i]);
            //            }
            //            currentLambda_ = min_dii;
            //        }
            currentLambda_ = currentLambda_ / beta;
            return false;
        }
    }

    /** @brief conjugate gradient with perconditioning
 *
 *  the jacobi PCG method
 *
 */
    VecX Problem::PCGSolver(const MatXX &A, const VecX &b, int maxIter = -1)
    {
        assert(A.rows() == A.cols() && "PCG solver ERROR: A is not a square matrix");

        int rows = b.rows();
        int n = maxIter < 0 ? rows : maxIter;
        VecX x(VecX::Zero(rows));
        MatXX M_inv = A.diagonal().asDiagonal().inverse();
        VecX r0(b); // initial r = b - A*0 = b
        VecX z0 = M_inv * r0;
        VecX p(z0);
        VecX w = A * p;
        double r0z0 = r0.dot(z0);
        double alpha = r0z0 / p.dot(w);
        VecX r1 = r0 - alpha * w;
        int i = 0;
        double threshold = 1e-6 * r0.norm();
        while (r1.norm() > threshold && i < n)
        {
            i++;
            VecX z1 = M_inv * r1;
            double r1z1 = r1.dot(z1);
            double belta = r1z1 / r0z0;
            z0 = z1;
            r0z0 = r1z1;
            r0 = r1;
            p = belta * p + z1;
            w = A * p;
            alpha = r1z1 / p.dot(w);
            x += alpha * p;
            r1 -= alpha * w;
        }
        return x;
    }

    /*
 *  marg 所有和 frame 相连的 edge: imu factor, projection factor
 *  如果某个landmark和该frame相连，但是又不想加入marg, 那就把改edge先去掉
 *
 */
    bool Problem::Marginalize(const std::vector<std::shared_ptr<Vertex>> margVertexs, int pose_dim)
    {
        SetOrdering();
        /// 找到需要 marg 的 edge, margVertexs[0] is frame, its edge contained pre-intergration
        std::vector<shared_ptr<Edge>> marg_edges = GetConnectedEdges(margVertexs[0]);

        std::unordered_map<int, shared_ptr<Vertex>> margLandmark;
        // 构建 Hessian 的时候 pose 的顺序不变，landmark的顺序要重新设定
        int marg_landmark_size = 0;
        //    std::cout << "\n marg edge 1st id: "<< marg_edges.front()->Id() << " end id: "<<marg_edges.back()->Id()<<'\n';
        for (size_t i = 0; i < marg_edges.size(); ++i)
        {
            //        std::cout << "marg edge id: "<< marg_edges[i]->Id() <<'\n';
            auto verticies = marg_edges[i]->Verticies();
            for (auto iter : verticies)
            {
                //margin landmark  when the marginal vertex connect edge
                if (IsLandmarkVertex(iter) && margLandmark.find(iter->Id()) == margLandmark.end())
                {
                    iter->SetOrderingId(pose_dim + marg_landmark_size);
                    margLandmark.insert(make_pair(iter->Id(), iter));
                    marg_landmark_size += iter->LocalDimension();
                }
            }
        }
        //    std::cout << "pose dim: " << pose_dim <<'\n';
        int cols = pose_dim + marg_landmark_size;
        /// 构建误差 H 矩阵 H = H_marg + H_pp_pprior
        MatXX H_marg(MatXX::Zero(cols, cols));
        VecX b_marg(VecX::Zero(cols));
        int ii = 0;
        for (auto edge : marg_edges)
        {
            edge->ComputeResidual();
            edge->ComputeJacobians();
            auto jacobians = edge->Jacobians();
            auto verticies = edge->Verticies();
            ii++;

            assert(jacobians.size() == verticies.size());
            for (size_t i = 0; i < verticies.size(); ++i)
            {
                auto v_i = verticies[i];
                auto jacobian_i = jacobians[i];
                ulong index_i = v_i->OrderingId();
                ulong dim_i = v_i->LocalDimension();

                double drho;
                MatXX robustInfo(edge->Information().rows(), edge->Information().cols());
                edge->RobustInfo(drho, robustInfo); //根據殘差, 訊息矩陣, lossfunction去調整訊息矩陣以及對應的調整參數

                for (size_t j = i; j < verticies.size(); ++j)
                {
                    auto v_j = verticies[j];
                    auto jacobian_j = jacobians[j];
                    ulong index_j = v_j->OrderingId();
                    ulong dim_j = v_j->LocalDimension();

                    MatXX hessian = jacobian_i.transpose() * robustInfo * jacobian_j;

                    assert(hessian.rows() == v_i->LocalDimension() && hessian.cols() == v_j->LocalDimension());
                    // 所有的信息矩阵叠加起来
                    H_marg.block(index_i, index_j, dim_i, dim_j) += hessian;
                    if (j != i)
                    {
                        // 对称的下三角
                        H_marg.block(index_j, index_i, dim_j, dim_i) += hessian.transpose();
                    }
                }
                b_marg.segment(index_i, dim_i) -= drho * jacobian_i.transpose() * edge->Information() * edge->Residual();
            }
        }
#if DEBUG_LOG_
        std::cout << "edge factor cnt: " << ii << '\n';
#endif
        /// marg landmark
        int reserve_size = pose_dim;
        if (marg_landmark_size > 0)
        {
            int marg_size = marg_landmark_size;
            MatXX Hmm = H_marg.block(reserve_size, reserve_size, marg_size, marg_size);
            MatXX Hpm = H_marg.block(0, reserve_size, reserve_size, marg_size);
            MatXX Hmp = H_marg.block(reserve_size, 0, marg_size, reserve_size);
            VecX bpp = b_marg.segment(0, reserve_size);
            VecX bmm = b_marg.segment(reserve_size, marg_size);

            // Hmm 是对角线矩阵，它的求逆可以直接为对角线块分别求逆，如果是逆深度，对角线块为1维的，则直接为对角线的倒数，这里可以加速
            MatXX Hmm_inv(MatXX::Zero(marg_size, marg_size));
            // TODO:: use openMP
            for (auto iter : margLandmark)
            {
                int idx = iter.second->OrderingId() - reserve_size;
                int size = iter.second->LocalDimension();
                Hmm_inv.block(idx, idx, size, size) = Hmm.block(idx, idx, size, size).inverse();
            }

            MatXX tempH = Hpm * Hmm_inv;
            MatXX Hpp = H_marg.block(0, 0, reserve_size, reserve_size) - tempH * Hmp;
            bpp = bpp - tempH * bmm;
            H_marg = Hpp;
            b_marg = bpp;
        }

        VecX b_prior_before = b_prior_;
        if (H_prior_.rows() > 0)
        {
            H_marg += H_prior_;
            b_marg += b_prior_;
        }

        /// marg frame and speedbias
        int marg_dim = 0;

        // index 大的先移动
        for (int k = margVertexs.size() - 1; k >= 0; --k)
        {
            int idx = margVertexs[k]->OrderingId();
            int dim = margVertexs[k]->LocalDimension();
            //        std::cout << k << " "<<idx << '\n';
            marg_dim += dim;
            // move the marg pose to the Hmm bottown right
            // 将 row i 移动矩阵最下面
            Eigen::MatrixXd temp_rows = H_marg.block(idx, 0, dim, reserve_size);
            Eigen::MatrixXd temp_botRows = H_marg.block(idx + dim, 0, reserve_size - idx - dim, reserve_size);
            H_marg.block(idx, 0, reserve_size - idx - dim, reserve_size) = temp_botRows;
            H_marg.block(reserve_size - dim, 0, dim, reserve_size) = temp_rows;

            // 将 col i 移动矩阵最右边
            Eigen::MatrixXd temp_cols = H_marg.block(0, idx, reserve_size, dim);
            Eigen::MatrixXd temp_rightCols = H_marg.block(0, idx + dim, reserve_size, reserve_size - idx - dim);
            H_marg.block(0, idx, reserve_size, reserve_size - idx - dim) = temp_rightCols;
            H_marg.block(0, reserve_size - dim, reserve_size, dim) = temp_cols;

            Eigen::VectorXd temp_b = b_marg.segment(idx, dim);
            Eigen::VectorXd temp_btail = b_marg.segment(idx + dim, reserve_size - idx - dim);
            b_marg.segment(idx, reserve_size - idx - dim) = temp_btail;
            b_marg.segment(reserve_size - dim, dim) = temp_b;
        }

        double eps = 1e-8;
        int m2 = marg_dim;
        int n2 = reserve_size - marg_dim; // marg pose

        //The purpose of window sliding is for smoothing, when the value is directly close to 0, there is not smoothing effect.
        Eigen::MatrixXd Amm = 0.5 * (H_marg.block(n2, n2, m2, m2) + H_marg.block(n2, n2, m2, m2).transpose());
        Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> saes(Amm);
        Eigen::MatrixXd Amm_inv = saes.eigenvectors() * Eigen::VectorXd((saes.eigenvalues().array() > eps).select(saes.eigenvalues().array().inverse(), 0)).asDiagonal() *
                                  saes.eigenvectors().transpose();

        Eigen::VectorXd bmm2 = b_marg.segment(n2, m2);
        Eigen::MatrixXd Arm = H_marg.block(0, n2, n2, m2);
        Eigen::MatrixXd Amr = H_marg.block(n2, 0, m2, n2);
        Eigen::MatrixXd Arr = H_marg.block(0, 0, n2, n2);
        Eigen::VectorXd brr = b_marg.segment(0, n2);
        Eigen::MatrixXd tempB = Arm * Amm_inv;
        H_prior_ = Arr - tempB * Amr; // H_prior_ mean margin first pose componment (Amm is margin pose)
        b_prior_ = brr - tempB * bmm2;

        Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> saes2(H_prior_);
        Eigen::VectorXd S = Eigen::VectorXd((saes2.eigenvalues().array() > eps).select(saes2.eigenvalues().array(), 0));
        Eigen::VectorXd S_inv = Eigen::VectorXd(
            (saes2.eigenvalues().array() > eps).select(saes2.eigenvalues().array().inverse(), 0));

        Eigen::VectorXd S_sqrt = S.cwiseSqrt();
        Eigen::VectorXd S_inv_sqrt = S_inv.cwiseSqrt();
        Jt_prior_inv_ = S_inv_sqrt.asDiagonal() * saes2.eigenvectors().transpose();
        err_prior_ = -Jt_prior_inv_ * b_prior_;
        
        MatXX J = S_sqrt.asDiagonal() * saes2.eigenvectors().transpose();
        Jt_prior_ = J;
        H_prior_ = J.transpose() * J;
        MatXX tmp_h = MatXX((H_prior_.array().abs() > 1e-9).select(H_prior_.array(), 0));
        H_prior_ = tmp_h;

        // std::cout << "my marg b prior: " <<b_prior_.rows()<<" norm: "<< b_prior_.norm() << '\n';
        // std::cout << "    error prior: " <<err_prior_.norm() << '\n';

        // remove vertex and remove edge
        for (size_t k = 0; k < margVertexs.size(); ++k)
        {
            RemoveVertex(margVertexs[k]);
        }

        for (auto landmarkVertex : margLandmark)
        {
            RemoveVertex(landmarkVertex.second);
        }

        return true;
    }

} // namespace Vsolver
