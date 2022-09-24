#include "fgo_gps_imu/fgo_gps_imu_solver.h"

// 0 : ceres solver, 1 : vsolver, 2: template ba_solver
#define CERES_SOLVE_BULL_IMU 0

FGO_GpsIMU_SOLVER::FGO_GpsIMU_SOLVER(ParamReaderPtr& slam_parameter)
{
  std::cout << "-------BackEnd Init-----------" << '\n';
  // TODO:: default fx for evluation normailze distance
  gyr_n = slam_parameter->gyr_arw;
  acc_n = slam_parameter->acc_vrw;
  gyr_w = slam_parameter->gyr_bias_std;
  acc_w = slam_parameter->acc_bias_std;
  gnss_window_size = slam_parameter->gnss_window_size;
  state = Gw_NEED_INIT;

  gnss_imu_time_delay = 0;

  // IMU noise
  gyr_noise_cov = gyr_n * gyr_n * Eigen::Matrix3d::Identity();
  acc_noise_cov = acc_n * acc_n * Eigen::Matrix3d::Identity();
  gyr_bias_cov = gyr_w * gyr_w * Eigen::Matrix3d::Identity();
  acc_bias_cov = acc_w * acc_w * Eigen::Matrix3d::Identity();

  std::cout << "FGO_GpsIMU_SOLVER gyr_n" << gyr_n << std::endl;
  std::cout << "FGO_GpsIMU_SOLVER acc_n" << acc_n << std::endl;
  std::cout << "FGO_GpsIMU_SOLVER gyr_w" << gyr_w << std::endl;
  std::cout << "FGO_GpsIMU_SOLVER acc_w" << acc_w << std::endl;

  para_bias = new double[(gnss_window_size + 1) * 6];
  para_pose = new double[(gnss_window_size + 1) * 7];
  para_speed = new double[(gnss_window_size + 1) * 3];

  request_reset_flag = false;

  //*NOTE: Init PriorTerm Info.
  Hprior_ = MatXX(0, 0);
  Jprior_inv_ = MatXX(0, 0);
  Jprior_ = MatXX(0, 0);
  bprior_ = VecX(0);
  errprior_ = VecX(0);

  //*NOTE: Template Solver
  prior_H.resize(0, 0);
  prior_JTinv.resize(0, 0);
  prior_b.resize(0);
  prior_r.resize(0);
  zupt_acc_norm.clear();
  d_gnss_frames.clear();
  gnss_system_initial_timestamp = 0;
  auto_detect_imu_fps = -1;
  station_origin_llh.setZero();
  tmp_lever_arm.setZero();
  initGPS = false;
}


FGO_GpsIMU_SOLVER::FGO_GpsIMU_SOLVER(double gyr_arw, double acc_vrw,
                                     double gyr_bias_std, double acc_bias_std,
                                     int window_size_gnss)
{
  std::cout << "-------BackEnd Init-----------" << '\n';
  // TODO:: default fx for evluation normailze distance
  gyr_n = gyr_arw;
  acc_n = acc_vrw;
  gyr_w = gyr_bias_std;
  acc_w = acc_bias_std;
  gnss_window_size = window_size_gnss;
  state = Gw_NEED_INIT;

  para_bias = new double[(gnss_window_size + 1) * 6];
  para_pose = new double[(gnss_window_size + 1) * 7];
  para_speed = new double[(gnss_window_size + 1) * 3];

  request_reset_flag = false;

  //*NOTE: Init PriorTerm Info.
  Hprior_ = MatXX(0, 0);
  Jprior_inv_ = MatXX(0, 0);
  Jprior_ = MatXX(0, 0);
  bprior_ = VecX(0);
  errprior_ = VecX(0);

  //*NOTE: Template Solver
  prior_H.resize(0, 0);
  prior_JTinv.resize(0, 0);
  prior_b.resize(0);
  prior_r.resize(0);
  zupt_acc_norm.clear();
  d_gnss_frames.clear();
  gnss_system_initial_timestamp = 0;
  auto_detect_imu_fps = -1;
  station_origin_llh.setZero();
  tmp_lever_arm.setZero();
  initGPS = false;
}


FGO_GpsIMU_SOLVER::~FGO_GpsIMU_SOLVER()
{
  // Rest InitValue
  state = Gw_NEED_INIT;
  gnss_system_initial_timestamp = 0;
  d_gnss_frames.clear();

  //*NOTE: Ori-Vsolver
  Hprior_ = MatXX(0, 0);
  Jprior_inv_ = MatXX(0, 0);
  Jprior_ = MatXX(0, 0);
  bprior_ = VecX(0);
  errprior_ = VecX(0);

  //*NOTE: Template Solver
  prior_H.resize(0, 0);
  prior_JTinv.resize(0, 0);
  prior_b.resize(0);
  prior_r.resize(0);

  zupt_acc_norm.clear();
  auto_detect_imu_fps = -1;
  station_origin_llh.setZero();
  initGPS = false;

  //*NOTE: ceres only
  last_marginalization_info_para_block.clear();
  last_margin_info = nullptr;
}

FGO_GpsIMU_SOLVER::GNSSResult FGO_GpsIMU_SOLVER::ProcessGNSSIMU(
  GNSSFramePtr& cur_frame)
{

  GNSSResult result;
  result.cur_frame_result = cur_frame;

  // llh convert to local pose as update measurement for FGO
  if (cur_frame->ori_blh.x() != 0 || cur_frame->ori_blh.y() != 0) {
    cur_frame->have_gnss_info = true;
    if (state == Gw_NEED_INIT) {
      double gw_magnitude = FGO_GpsIMU_SOLVER::gravity(cur_frame->ori_blh);
      Gw = Eigen::Vector3d(0.0, 0.0, gw_magnitude);
      state = Acc_Need_INIT_Rwb;
    }
  } else {
    cur_frame->station2cur_blh_local_data.setZero();
    if (state == Gw_NEED_INIT) {
      Reset();
      return result;
    }
  }

  if (state == Acc_Need_INIT_Rwb && (cur_frame->have_gnss_info)) {
    // NOTE::Step1 First Get GNSS info LLH(BLH), get correspondence gravity
    int result_static_init = InitEstAccRwb(cur_frame);
    if (result_static_init == 1) {
      std::cout << "\033[1;31mbold Acc_Need_INIT_RwbInitSucess\033[0m" << std::endl;
      state = WAIT_FILL_WINDOW;
      result.state = state;
      return result;
    } else {
      result.state = state;
      return result;
    }
  }
  else if (state == WAIT_FILL_WINDOW) {
    if (cur_frame->have_gnss_info) {
      cur_frame->station2cur_blh_local_data =
        global2local(station_origin_llh, cur_frame->ori_blh);
    }
    std::cout << "==WAIT_FILL_WINDOW== " <<std::endl;
    std::cout << "blh_measurement=" << cur_frame->station2cur_blh_local_data << std::endl;
    fgo_marginal_flag = MARGIN_OLD;
    d_gnss_frames.emplace_back(cur_frame);
    GNSSFramePtr ref_frame = *(d_gnss_frames.end() - 2);
    PredictNextGNSSFrameImu(ref_frame, cur_frame);

#if CERES_SOLVE_BULL_IMU == 0
    SolveFGOCeres();
#elif CERES_SOLVE_BULL_IMU == 1
    SolveFGO();
#else
    SolveBATemplate();
#endif

    if (d_gnss_frames.size() == gnss_window_size + 1) {
#if CERES_SOLVE_BULL_IMU == 0
      MarginalizeCeres();
#elif CERES_SOLVE_BULL_IMU == 1
      MarginalizeGlobalInfo();
#else
      MarginalizeBATemplate();
#endif
      SlidingWindow();
      state = TIGHTLY_GINS;
    }
    result.cur_frame_result = d_gnss_frames.back();
  } else if (state == TIGHTLY_GINS) {
    if (cur_frame->have_gnss_info) {
      cur_frame->station2cur_blh_local_data =
        global2local(station_origin_llh, cur_frame->ori_blh);
    }
    fgo_marginal_flag = MARGIN_OLD;

    d_gnss_frames.emplace_back(cur_frame);
    GNSSFramePtr ref_frame = *(d_gnss_frames.end() - 2);
    PredictNextGNSSFrameImu(ref_frame, cur_frame);

#if CERES_SOLVE_BULL_IMU == 0
    SolveFGOCeres();
    MarginalizeCeres();
#elif CERES_SOLVE_BULL_IMU == 1
    SolveFGO();
    MarginalizeGlobalInfo();
#else
    SolveBATemplate();
    MarginalizeBATemplate();
#endif
    SlidingWindow();
    result.cur_frame_result = d_gnss_frames.back();
  }

  result.state = state;
  return result;
}

void FGO_GpsIMU_SOLVER::PredictLastFrameOnly(double timestamp,
                                             const Eigen::Vector3d& acc,
                                             const Eigen::Vector3d& gyr)
{
  std::cout << "PredictLastFrameOnly=" << std::endl;
  GNSSFramePtr ref_frame = *(d_gnss_frames.end() - 1);

  Eigen::Vector3d gyr_0 = ref_frame->v_gyr.back(),
                  acc_0 = ref_frame->v_acc.back();
  double t0 = ref_frame->v_imu_timestamp.back();

  {
    double t = timestamp, dt = t - t0;
    ref_frame->imupreinte->push_back(dt, acc, gyr);

    // incremental angle (rad) / incremental velocity (m/s); raw imu data need
    // dot dt, from rad/s,m/s convert to rad,m
    Eigen::Vector3d imu_cur_dvel = (acc - ref_frame->ba) * dt;
    Eigen::Vector3d imu_pre_dvel = (acc_0 - ref_frame->ba) * dt;
    Eigen::Vector3d imu_cur_dtheta = (gyr - ref_frame->bg) * dt;
    Eigen::Vector3d imu_pre_dtheta = (gyr_0 - ref_frame->bg) * dt;

    // std::cout << "vsolver out imu_cur_dvel=" << imu_cur_dvel << std::endl;
    // std::cout << "vsolver out imu_pre_dvel=" << imu_pre_dvel << std::endl;
    // std::cout << "vsolver out imu_cur_dtheta=" << imu_cur_dtheta <<
    // std::endl; std::cout << "vsolver out imu_pre_dtheta=" << imu_pre_dtheta
    // << std::endl; std::cout << "vsolver Gw=" << Gw << std::endl;
    // 连续状态积分, 先位置速度再姿态
    // 位置速度

    Eigen::Vector3d dvfb = imu_cur_dvel +
      0.5 * imu_cur_dtheta.cross(imu_cur_dvel) +
      1.0 / 12.0 *
        (imu_pre_dtheta.cross(imu_cur_dvel) +
         imu_pre_dvel.cross(imu_cur_dtheta));
    // world gps is NED coordinate, gravity in acc is positive, we
    Eigen::Vector3d dvel = (ref_frame->q_wb * dvfb) - (Gw * dt);
    ref_frame->p_wb += dt * ref_frame->v_wb + (0.5 * dt * dvel);
    ref_frame->v_wb += dvel;

    // 姿态
    Eigen::Vector3d dtheta =
      (imu_cur_dtheta + 1.0 / 12.0 * imu_pre_dtheta.cross(imu_cur_dtheta));
    ref_frame->q_wb = ref_frame->q_wb * Sophus::SO3d::exp(dtheta);
    // std::cout << "vsolver t=" << t << std::endl;
    // std::cout << "vsolver t0=" << t0 << std::endl;
    // std::cout << "vsolver dt=" << dt << std::endl;
    // std::cout << "vsolver dtheta=" << dtheta << std::endl;
    // std::cout << "vsolver dvfb=" << dvfb << std::endl;

    ref_frame->v_acc.emplace_back(acc);
    ref_frame->v_gyr.emplace_back(gyr);
    ref_frame->v_imu_timestamp.emplace_back(timestamp);
  }
}

void FGO_GpsIMU_SOLVER::PredictNextGNSSFrameImu(const GNSSFramePtr& ref_frame,
                                                GNSSFramePtr& cur_frame)
{
  std::cout << "PredictNextGNSSFrameImu" << std::endl;

  cur_frame->q_wb = ref_frame->q_wb;
  cur_frame->p_wb = ref_frame->p_wb;
  cur_frame->v_wb = ref_frame->v_wb;
  cur_frame->ba = ref_frame->ba;
  cur_frame->bg = ref_frame->bg;
  cur_frame->imupreinte = std::make_shared<BullIntegrationBase>(
    ref_frame->v_acc.back(), ref_frame->v_gyr.back(), ref_frame->ba,
    ref_frame->bg, acc_n, gyr_n, acc_w, gyr_w);

  Eigen::Vector3d gyr_0 = ref_frame->v_gyr.back(),
                  acc_0 = ref_frame->v_acc.back();
  double t0 = ref_frame->v_imu_timestamp.back();
  std::cout << "cur_frame->v_acc.size()=" << cur_frame->v_acc.size()
            << std::endl;

  for (int i = 0, n = cur_frame->v_acc.size(); i < n; ++i) {

    double t = cur_frame->v_imu_timestamp[i], dt = t - t0;
    if(dt <= 0){
      continue;
    }

    Eigen::Vector3d gyr = cur_frame->v_gyr[i];
    Eigen::Vector3d acc = cur_frame->v_acc[i];
    cur_frame->imupreinte->push_back(dt, acc, gyr);

    // incremental angle (rad) / incremental velocity (m/s); raw imu data need
    // dot dt, from rad/s,m/s convert to rad,m
    Eigen::Vector3d imu_cur_dvel = (acc - ref_frame->ba) * dt;
    Eigen::Vector3d imu_pre_dvel = (acc_0 - ref_frame->ba) * dt;
    Eigen::Vector3d imu_cur_dtheta = (gyr - ref_frame->bg) * dt;
    Eigen::Vector3d imu_pre_dtheta = (gyr_0 - ref_frame->bg) * dt;

 

    // 连续状态积分, 先位置速度再姿态
    // 位置速度
    Eigen::Vector3d dvfb = imu_cur_dvel +
      0.5 * imu_cur_dtheta.cross(imu_cur_dvel) +
      1.0 / 12.0 *
        (imu_pre_dtheta.cross(imu_cur_dvel) +
         imu_pre_dvel.cross(imu_cur_dtheta));
    Eigen::Vector3d dvel = (cur_frame->q_wb * dvfb) - (Gw * dt);
    cur_frame->p_wb += dt * cur_frame->v_wb + (0.5 * dt * dvel);
    cur_frame->v_wb += dvel;

    // 姿态
    Eigen::Vector3d dtheta =
      (imu_cur_dtheta + 1.0 / 12.0 * imu_pre_dtheta.cross(imu_cur_dtheta));
    cur_frame->q_wb = cur_frame->q_wb * Sophus::SO3d::exp(dtheta);

    // std::cout << "dt=" << dt <<std::endl;
    // std::cout << "cur_frame->p_wb=" << cur_frame->p_wb << std::endl;
    // std::cout << "cur_frame->v_wb=" << cur_frame->v_wb << std::endl;

    gyr_0 = gyr;
    acc_0 = acc;
    t0 = t;
  }
}

void FGO_GpsIMU_SOLVER::data2double()
{
  for (int i = 0, n = d_gnss_frames.size(); i < n; ++i) {
    size_t idx_pose = i * 7;
    size_t idx_v = i * 3;
    size_t idx_b = i * 6;
    std::memcpy(para_pose + idx_pose, d_gnss_frames[i]->q_wb.data(),
                sizeof(double) * Sophus::SO3d::num_parameters);
    std::memcpy(para_pose + idx_pose + 4, d_gnss_frames[i]->p_wb.data(),
                sizeof(double) * 3);
    std::memcpy(para_speed + idx_v, d_gnss_frames[i]->v_wb.data(),
                sizeof(double) * 3);
    std::memcpy(para_bias + idx_b, d_gnss_frames[i]->ba.data(),
                sizeof(double) * 3);
    std::memcpy(para_bias + idx_b + 3, d_gnss_frames[i]->bg.data(),
                sizeof(double) * 3);
  }
}

void FGO_GpsIMU_SOLVER::double2data()
{
  Sophus::SO3d q_w0b0 = d_gnss_frames[0]->q_wb;
  Eigen::Vector3d p_w0b0 = d_gnss_frames[0]->p_wb;
  bool gps_info_window = false;

  for (int i = 0, n = d_gnss_frames.size(); i < n; ++i) {
    if (d_gnss_frames[i]->have_gnss_info) {
      gps_info_window = true;
    }
    size_t idx_pose = i * 7;
    size_t idx_v = i * 3;
    size_t idx_b = i * 6;
    std::memcpy(d_gnss_frames[i]->q_wb.data(), para_pose + idx_pose,
                sizeof(double) * Sophus::SO3d::num_parameters);
    std::memcpy(d_gnss_frames[i]->p_wb.data(), para_pose + idx_pose + 4,
                sizeof(double) * 3);
    std::memcpy(d_gnss_frames[i]->v_wb.data(), para_speed + idx_v,
                sizeof(double) * 3);
    std::memcpy(d_gnss_frames[i]->ba.data(), para_bias + idx_b,
                sizeof(double) * 3);
    std::memcpy(d_gnss_frames[i]->bg.data(), para_bias + idx_b + 3,
                sizeof(double) * 3);
  }

  // not good result when gps info enough or prior with gps info.
  /**
  if(!gps_info_window)
  {
      // when we have prior from IMU constriant,
      Sophus::SO3d q_w1b0 = d_gnss_frames[0]->q_wb;
      double y_diff = Sophus::R2ypr(q_w0b0 * q_w1b0.inverse())(0);
      Sophus::SO3d q_w0w1 = Sophus::ypr2R<double>(y_diff, 0, 0);
      Eigen::Vector3d p_w1b0 = d_gnss_frames[0]->p_wb;
      for (int i = 0, n = d_gnss_frames.size(); i < n; ++i)
      {
          d_gnss_frames[i]->q_wb = q_w0w1 * d_gnss_frames[i]->q_wb;
          d_gnss_frames[i]->p_wb = q_w0w1 * (d_gnss_frames[i]->p_wb - p_w1b0) +
  p_w0b0; d_gnss_frames[i]->v_wb = q_w0w1 * d_gnss_frames[i]->v_wb;
      }
  }
  */
}

bool FGO_GpsIMU_SOLVER::IMURawCheckZUPT()
{
  int count = zupt_acc_norm.size();
  if (count > auto_detect_imu_fps * 1.5) {
    double sum1_acc = 0; // sum of the numbers
    double sum2_acc = 0; // sum of the squares
    double min_acc_norm = 1000;
    double max_acc_norm = -1000;
    for (int i = 0, n = count; i < n; ++i) {
      sum1_acc += zupt_acc_norm[i];
      sum2_acc += zupt_acc_norm[i] * zupt_acc_norm[i];
      if (min_acc_norm > zupt_acc_norm[i]) {
        min_acc_norm = zupt_acc_norm[i];
      }

      if (max_acc_norm < zupt_acc_norm[i]) {
        max_acc_norm = zupt_acc_norm[i];
      }
    }
    double average_acc = sum1_acc / count;
    double variance_acc =
      ((count * sum2_acc) - sum1_acc * sum1_acc) / (count * count);
    double stdev_acc = std::sqrt(variance_acc);

    // std::cout << "1 count=" << count <<std::endl;
    int remove_idx = count * 0.8;
    zupt_acc_norm.erase(zupt_acc_norm.begin(),
                        zupt_acc_norm.begin() + remove_idx);
    // std::cout << " 2 count=" << count <<std::endl;

    if (stdev_acc < STD_stable_Th[0] &&
        (max_acc_norm - min_acc_norm) < STD_stable_Th[1]) {
      std::cout << "Zupt" << std::endl;
      return true;
    } else {
      // std::cout << "move" << std::endl;
      return false;
    }
  }
  return false;
}

int FGO_GpsIMU_SOLVER::InitEstAccRwb(GNSSFramePtr& frame)
{
  if (frame->v_imu_timestamp.size() < 5) {
    return 0;
  } else {
    // Step1. estimate std, nean of acc for check static motion.
    double sum1_acc = 0;  // sum of the numbers
    double sum2_acc = 0;  // sum of the squares
    double sum1_gyro = 0; // sum of the numbers
    double sum2_gyro = 0; // sum of the squares
    int count = frame->v_imu_timestamp.size();
    double min_acc_norm = 1000;
    double max_acc_norm = -1000;
    Eigen::Vector3d avg_acc_raw_data(0, 0, 0);
    for (int i = 0, n = count; i < n; ++i) {
      double gyr_norm = frame->v_gyr[i].norm();
      double acc_norm = frame->v_acc[i].norm();
      sum1_gyro += gyr_norm;
      sum2_gyro += gyr_norm * gyr_norm;
      sum1_acc += acc_norm;
      sum2_acc += acc_norm * acc_norm;
      avg_acc_raw_data += frame->v_acc[i];

      if (min_acc_norm > acc_norm) {
        min_acc_norm = acc_norm;
      }
      if (max_acc_norm < acc_norm) {
        max_acc_norm = acc_norm;
      }
    }

    avg_acc_raw_data = avg_acc_raw_data / count;
    double average_acc = sum1_acc / count;
    double variance_acc =
      ((count * sum2_acc) - sum1_acc * sum1_acc) / (count * count);
    double stdev_acc = std::sqrt(variance_acc);
    std::cout << "min_acc_norm) :" << min_acc_norm << std::endl;
    std::cout << "max_acc_norm() :" << max_acc_norm << std::endl;
    std::cout << "(max_acc_norm - min_acc_norm):"
              << (max_acc_norm - min_acc_norm) << std::endl;
    std::cout << "mat.mean() :" << average_acc << std::endl;
    std::cout << "variance() :" << variance_acc << std::endl;
    std::cout << "std_dev() :" << stdev_acc << std::endl;
    std::cout << "stdev_acc <STD_stable_Th[0]=" << STD_stable_Th[0]
              << std::endl;
    std::cout << "(max_acc_norm - min_acc_norm)  STD_stable_Th[1]="
              << STD_stable_Th[1] << std::endl;
    // Initialize the initial orientation, so that the estimation
    // is consistent with the inertial frame.
    // Step2. check static motion
    if (stdev_acc < STD_stable_Th[0] && (max_acc_norm - min_acc_norm) < STD_stable_Th[1])
     {
      std::cout << "StaticInitSucess" << std::endl;
      // Compute rotation.
      // Please refer to
      // https://github.com/rpng/open_vins/blob/master/ov_core/src/init/InertialInitializer.cpp

      // Three axises of the ENU frame in the IMU frame.
      // z-axis.
      const Eigen::Vector3d& z_axis = avg_acc_raw_data.normalized();
      // x-axis.
      Eigen::Vector3d x_axis = Eigen::Vector3d::UnitX() -
        z_axis * z_axis.transpose() * Eigen::Vector3d::UnitX();
      x_axis.normalize();
      // y-axis.
      Eigen::Vector3d y_axis = z_axis.cross(x_axis);
      y_axis.normalize();
      Eigen::Matrix3d I_R_G;
      I_R_G.block<3, 1>(0, 0) = x_axis;
      I_R_G.block<3, 1>(0, 1) = y_axis;
      I_R_G.block<3, 1>(0, 2) = z_axis;
      Eigen::Quaterniond tmp_Rwb(I_R_G.transpose());
      tmp_Rwb.normalize();

      // TODO:: set init state and create frame to d_frames
      gnss_system_initial_timestamp = frame->timestamp; // set first frame
      //[TODO]
      frame->q_wb.setQuaternion(tmp_Rwb);
      std::cout << "init qwb=" << frame->q_wb.unit_quaternion().coeffs() <<std::endl;
      frame->station2cur_blh_local_data.setZero();
      frame->p_wb.setZero();
      frame->v_wb.setZero();
      frame->ba.setZero();
      frame->bg.setZero();
      frame->imupreinte = std::make_shared<BullIntegrationBase>(
        frame->v_acc[0], frame->v_gyr[0], frame->ba, frame->bg, acc_n, gyr_n,
        acc_w, gyr_w);
      d_gnss_frames.emplace_back(frame);
      station_origin_llh = frame->ori_blh;
      std::cout << "frame->q_wb * acc_avg=" << frame->q_wb * avg_acc_raw_data
                << std::endl;
      return 1;
    } 
    else {
      return 0;
    }
  }
}

void FGO_GpsIMU_SOLVER::SolveFGO()
{
  data2double();
  Vsolver::LossFunction* lossfunction;
  // lossfunction = new Vsolver::CauchyLoss(cv_huber_loss_parameter);

  // step1. 构建 problem
  Vsolver::Problem problem(Vsolver::Problem::ProblemType::SLAM_PROBLEM);
  std::vector<std::shared_ptr<Vsolver::VertexPose>> vertexCams_vec;
  std::vector<std::shared_ptr<Vsolver::VertexSpeed>> vertexV_vec;
  std::vector<std::shared_ptr<Vsolver::VertexBias>> vertexBias_vec;
  int pose_dim = 0;

  // TODO:: add time-delay factor

  bool first_frame_gps = d_gnss_frames[0]->have_gnss_info;
  // SlideWindow Vetrx of Frame info : PQV,Ba,Bg
  for (int i = 0, n = d_gnss_frames.size(); i < n; ++i) {
    std::shared_ptr<Vsolver::VertexPose> vertexCam(new Vsolver::VertexPose());
    Eigen::Map<Vec7> pose(para_pose + i * 7);
    vertexCam->SetParameters(pose);

    vertexCams_vec.emplace_back(vertexCam);
    problem.AddVertex(vertexCam);
    pose_dim += vertexCam->LocalDimension();
    // V / Ba /Bg
    std::shared_ptr<Vsolver::VertexSpeed> vertexVelocity(
      new Vsolver::VertexSpeed());
    std::shared_ptr<Vsolver::VertexBias> vertexBiasAG(
      new Vsolver::VertexBias());
    Eigen::Map<Vec6> bias_body_ag(para_bias + i * 6);
    Eigen::Map<Vec3> vel_body(para_speed + i * 3);
    vertexVelocity->SetParameters(vel_body);
    vertexBiasAG->SetParameters(bias_body_ag);

    vertexV_vec.emplace_back(vertexVelocity);
    vertexBias_vec.emplace_back(vertexBiasAG);
    problem.AddVertex(vertexVelocity);
    pose_dim += vertexVelocity->LocalDimension();
    problem.AddVertex(vertexBiasAG);
    pose_dim += vertexBiasAG->LocalDimension();
  }

  int no_gps_factor = 0;
  // edge of IMU and Odom
  for (int i = 0, n = d_gnss_frames.size(); i < n; ++i) {
    if (d_gnss_frames[i]->have_gnss_info) {
      // std::cout << "gnss info=" << i << "/" << std::setprecision(20) <<
      // d_gnss_frames[i]->timestamp <<std::endl;
      std::cout << "Vsolver GNSS time=" << d_gnss_frames[i]->timestamp
                << std::endl;
      std::cout << "Vsolver GNSS value="
                << d_gnss_frames[i]->station2cur_blh_local_data << std::endl;
      std::shared_ptr<Vsolver::EdgeGNSS> GNSSEdge(new Vsolver::EdgeGNSS(
        d_gnss_frames[i]->station2cur_blh_local_data, tmp_lever_arm,
        d_gnss_frames[i]->gnss_llh_variance));
      std::vector<std::shared_ptr<Vsolver::Vertex>> edge_gnss_vertex;
      edge_gnss_vertex.emplace_back(vertexCams_vec[i]);
      GNSSEdge->SetVertex(edge_gnss_vertex);
      problem.AddEdge(GNSSEdge);
    }

    if (i == 0)
      continue;

    // imu measurement
    // TODO:: when long time gnss dissappear reset system. or increse newest
    // gps variance if (d_gnss_frames[i]->imupreinte->sum_dt < max_imu_sum_t)
    {
      // imu edge
      std::shared_ptr<Vsolver::EdgeBullImu> imuEdge(
        new Vsolver::EdgeBullImu(d_gnss_frames[i]->imupreinte, Gw));
      std::cout << "Vsolver IMU_Connect time=" << d_gnss_frames[i]->timestamp
                << std::endl;
      std::cout << "vsolver delta_state_.p="
                << d_gnss_frames[i]->imupreinte->delta_p << std::endl;
      std::cout << "vsolver delta_state_.v="
                << d_gnss_frames[i]->imupreinte->delta_v << std::endl;
      std::cout
        << "vsolver delta_state_.q="
        << d_gnss_frames[i]->imupreinte->delta_q.unit_quaternion().coeffs()
        << std::endl;
      std::cout << "vsolver current_state_.p=" << d_gnss_frames[i]->p_wb
                << std::endl;
      std::cout << "vsolver current_state_.v=" << d_gnss_frames[i]->v_wb
                << std::endl;
      std::cout << "vsolver current_state_.q="
                << d_gnss_frames[i]->q_wb.unit_quaternion().coeffs()
                << std::endl;
      std::vector<std::shared_ptr<Vsolver::Vertex>> edge_vertex;
      edge_vertex.emplace_back(vertexCams_vec[i - 1]);
      edge_vertex.emplace_back(vertexV_vec[i - 1]);
      edge_vertex.emplace_back(vertexBias_vec[i - 1]);
      edge_vertex.emplace_back(vertexCams_vec[i]);
      edge_vertex.emplace_back(vertexV_vec[i]);
      edge_vertex.emplace_back(vertexBias_vec[i]);
      imuEdge->SetVertex(edge_vertex);
      problem.AddEdge(imuEdge);
    }
  } // for (int i = 0, n = d_frames.size(); i < n; ++i)

  {
    std::shared_ptr<Vsolver::EdgeLastBias> last_bias_edge(
      new Vsolver::EdgeLastBias());
    std::vector<std::shared_ptr<Vsolver::Vertex>> edge_last_bias;
    edge_last_bias.emplace_back(vertexBias_vec[d_gnss_frames.size() - 1]);
    last_bias_edge->SetVertex(edge_last_bias);
    problem.AddEdge(last_bias_edge);
  }

  // 先验
  {
    // 已经有 Prior 了
    if (Hprior_.rows() > 0) {
      // 外参数先验设置为 0. TODO:: 这个应该放到 solver 里去弄
      // Hprior_.block(0,0,6,Hprior_.cols()).setZero();
      // Hprior_.block(0,0,Hprior_.rows(),6).setZero();
      problem.SetHessianPrior(Hprior_); // 告诉这个 problem
      problem.SetbPrior(bprior_);
      problem.SetErrPrior(errprior_);
      problem.SetJtPrior(Jprior_inv_);
      problem.SetJtPrior_nonInv(Jprior_);
      problem.ExtendHessiansPriorSize(
        15); // 但是这个 prior 还是之前的维度，需要扩展下装新的pose
    }
  }

  // NOTE Using DogLeg Method
  // problem.Solve(1);
  problem.SolveWithDogLeg(10);

  // update vertex
  // update bprior_,  Hprior_ do not need update
  if (Hprior_.rows() > 0) {
    bprior_ = problem.GetbPrior();
    errprior_ = problem.GetErrPrior();
#if INIT_DEBUG_COUT
    std::cout << "----------- update bprior -------------\n";
    std::cout << "             before: " << bprior_.norm() << '\n';
    std::cout << "                     " << errprior_.norm() << '\n';
    std::cout << "             after: " << bprior_.norm() << '\n';
    std::cout << "                    " << errprior_.norm() << '\n';
#endif
  }

  // update P Q V ba bg vertex
  for (int i = 0, n = d_gnss_frames.size(); i < n; ++i) {
    Vec7 p = vertexCams_vec[i]->Parameters();

    for (int j = 0; j < 7; ++j) {
      para_pose[(i * 7 + j)] = p[j];
    }

    Vec3 v = vertexV_vec[i]->Parameters();
    for (int j = 0; j < 3; ++j) {
      para_speed[i * 3 + j] = v[j];
    }

    Vec6 bias_ag = vertexBias_vec[i]->Parameters();
    for (int j = 0; j < 6; ++j) {
      para_bias[i * 6 + j] = bias_ag[j];
    }
  }

  double2data();
  // vsolver::Trace::TraceEnd();
}

void FGO_GpsIMU_SOLVER::MarginalizeGlobalInfo()
{
  if (fgo_marginal_flag == MARGIN_OLD) {
    //發現margin_old部份與solveBA的輸出相同,
    //因此將SolveBAImu輸出變成Marginalize使用於margin_old中減少計算
    //發現這部份與 solve_ba解出來的problem相同, 因此將solveBA結果輸入近來沿用
    data2double();
    Vsolver::LossFunction* lossfunction;
    lossfunction = new Vsolver::CauchyLoss(cv_huber_loss_parameter);
    // step1. 构建 problem
    Vsolver::Problem problem(Vsolver::Problem::ProblemType::SLAM_PROBLEM);
    std::vector<std::shared_ptr<Vsolver::VertexPose>> vertexCams_vec;
    std::vector<std::shared_ptr<Vsolver::VertexSpeed>> vertexV_vec;
    std::vector<std::shared_ptr<Vsolver::VertexBias>> vertexBias_vec;
    int pose_dim = 0;

    // SlideWindow Vetrx of Frame info : PQV,Ba,Bg
    for (int i = 0, n = d_gnss_frames.size(); i < n; ++i) {
      std::shared_ptr<Vsolver::VertexPose> vertexCam =
        std::make_shared<Vsolver::VertexPose>();
      Eigen::Map<Vec7> pose(para_pose + i * 7);
      vertexCam->SetParameters(pose);

      vertexCams_vec.emplace_back(vertexCam);
      problem.AddVertex(vertexCam);
      pose_dim += vertexCam->LocalDimension();

      std::shared_ptr<Vsolver::VertexSpeed> vertexVelocity =
        std::make_shared<Vsolver::VertexSpeed>();
      std::shared_ptr<Vsolver::VertexBias> vertexBiasAG =
        std::make_shared<Vsolver::VertexBias>();
      Eigen::Map<Vec6> bias_body_ag(para_bias + i * 6);
      Eigen::Map<Vec3> vel_body(para_speed + i * 3);
      vertexVelocity->SetParameters(vel_body);
      vertexBiasAG->SetParameters(bias_body_ag);
      vertexV_vec.emplace_back(vertexVelocity);
      vertexBias_vec.emplace_back(vertexBiasAG);
      problem.AddVertex(vertexVelocity);
      pose_dim += vertexVelocity->LocalDimension();
      problem.AddVertex(vertexBiasAG);
      pose_dim += vertexBiasAG->LocalDimension();
    }

    // edge of IMU and gnss
    {
      if (d_gnss_frames[0]->have_gnss_info) {
        std::shared_ptr<Vsolver::EdgeGNSS> GNSSEdge(new Vsolver::EdgeGNSS(
          d_gnss_frames[0]->station2cur_blh_local_data, tmp_lever_arm,
          d_gnss_frames[0]->gnss_llh_variance));
        std::vector<std::shared_ptr<Vsolver::Vertex>> edge_gnss_vertex;
        edge_gnss_vertex.emplace_back(vertexCams_vec[0]);
        GNSSEdge->SetVertex(edge_gnss_vertex);
        problem.AddEdge(GNSSEdge);
      }

      // imu info
      // if (d_gnss_frames[1]->imupreinte->sum_dt < max_imu_sum_t)
      {
        // imu edge
        std::shared_ptr<Vsolver::EdgeBullImu> imuEdge =
          std::make_shared<Vsolver::EdgeBullImu>(d_gnss_frames[1]->imupreinte,
                                                 Gw);
        std::vector<std::shared_ptr<Vsolver::Vertex>> edge_vertex;
        edge_vertex.emplace_back(vertexCams_vec[0]);
        edge_vertex.emplace_back(vertexV_vec[0]);
        edge_vertex.emplace_back(vertexBias_vec[0]);
        edge_vertex.emplace_back(vertexCams_vec[1]);
        edge_vertex.emplace_back(vertexV_vec[1]);
        edge_vertex.emplace_back(vertexBias_vec[1]);
        imuEdge->SetVertex(edge_vertex);
        problem.AddEdge(imuEdge);
      }

    } // edge of IMU and gnss--

    // 先验
    {
      // 已经有 Prior 了
      if (Hprior_.rows() > 0) {
        problem.SetHessianPrior(Hprior_); // 告诉这个 problem
        problem.SetbPrior(bprior_);
        problem.SetErrPrior(errprior_);
        problem.SetJtPrior(Jprior_inv_);
        problem.SetJtPrior_nonInv(Jprior_);
        problem.ExtendHessiansPriorSize(
          15); // 但是这个 prior 还是之前的维度，需要扩展下装新的pose
      } else {
        Hprior_ = MatXX(pose_dim, pose_dim);
        Hprior_.setZero();
        bprior_ = VecX(pose_dim);
        bprior_.setZero();
        problem.SetHessianPrior(Hprior_); // 告诉这个 problem
        problem.SetbPrior(bprior_);
      }
    }

    // Margin first frame vertex
    std::vector<std::shared_ptr<Vsolver::Vertex>> marg_vertex;
    marg_vertex.emplace_back(vertexCams_vec[0]);
    marg_vertex.emplace_back(vertexV_vec[0]);
    marg_vertex.emplace_back(vertexBias_vec[0]);
    problem.Marginalize(marg_vertex, pose_dim);
    Hprior_ = problem.GetHessianPrior();
    bprior_ = problem.GetbPrior();
    errprior_ = problem.GetErrPrior();
    Jprior_inv_ = problem.GetJtPrior();
    Jprior_ = problem.GetJtPrior_nonInv();
  } // margin_old_end

#if 0
    else if (Hprior_.rows() > 0) {
        // fgo_marginal_flag == MARGIN_NEW
        // margin_new and exist prior info.
        //  auto it = std::find(last_marginalization_info_para_block.begin(), last_marginalization_info_para_block.end(),
        //  para_pose + 7 * (d_frames.size() - 2)); *NOTE : d_frames[d_frames.size()
        //- 2]->frame_been_margin_old_state as
        {
            
            data2double();
            // step1. 构建 problem
            Vsolver::Problem problem(Vsolver::Problem::ProblemType::SLAM_PROBLEM);
            std::vector<std::shared_ptr<Vsolver::VertexPose>> vertexCams_vec;
            std::vector<std::shared_ptr<Vsolver::VertexSpeed>> vertexV_vec;
            std::vector<std::shared_ptr<Vsolver::VertexBias>> vertexBias_vec;
            int pose_dim = 0;

            // SlideWindow Vetrx of Frame info : PQV,Ba,Bg
            for (int i = 0, n = d_gnss_frames.size(); i < n; ++i) {
                std::shared_ptr<Vsolver::VertexPose> vertexCam = std::make_shared<Vsolver::VertexPose>();
                Eigen::Map<Vec7> pose(para_pose + i * 7);
                vertexCam->SetParameters(pose);
                vertexCams_vec.emplace_back(vertexCam);
                problem.AddVertex(vertexCam);
                pose_dim += vertexCam->LocalDimension();

                std::shared_ptr<Vsolver::VertexSpeed> vertexVelocity = std::make_shared<Vsolver::VertexSpeed>();
                std::shared_ptr<Vsolver::VertexBias> vertexBiasAG = std::make_shared<Vsolver::VertexBias>();
                Eigen::Map<Vec6> bias_body_ag(para_bias + i * 6);
                Eigen::Map<Vec3> vel_body(para_speed + i * 3);
                vertexVelocity->SetParameters(vel_body);
                vertexBiasAG->SetParameters(bias_body_ag);
                vertexV_vec.emplace_back(vertexVelocity);
                vertexBias_vec.emplace_back(vertexBiasAG);
                problem.AddVertex(vertexVelocity);
                pose_dim += vertexVelocity->LocalDimension();
                problem.AddVertex(vertexBiasAG);
                pose_dim += vertexBiasAG->LocalDimension();
            }

            // 先验
            {
                problem.SetHessianPrior(Hprior_); // 告诉这个 problem
                problem.SetbPrior(bprior_);
                problem.SetErrPrior(errprior_);
                problem.SetJtPrior(Jprior_inv_);
                problem.SetJtPrior_nonInv(Jprior_);
                problem.ExtendHessiansPriorSize(15); // 但是这个 prior 还是之前的维度，需要扩展下装新的pose
            }

            std::vector<std::shared_ptr<Vsolver::Vertex>> marg_vertex;
            // 把窗口倒数第二个帧 marg 掉
            marg_vertex.emplace_back(vertexCams_vec[d_gnss_frames.size() - 2]);
            marg_vertex.emplace_back(vertexV_vec[d_gnss_frames.size() - 2]);
            marg_vertex.emplace_back(vertexBias_vec[d_gnss_frames.size() - 2]);
            problem.Marginalize(marg_vertex, pose_dim);
            Hprior_ = problem.GetHessianPrior();
            bprior_ = problem.GetbPrior();
            errprior_ = problem.GetErrPrior();
            Jprior_inv_ = problem.GetJtPrior();
            Jprior_ = problem.GetJtPrior_nonInv();
        
        }
    }
#endif
}

void FGO_GpsIMU_SOLVER::SlidingWindow()
{
  // std::cout << "fgo_marginal_flag=" << fgo_marginal_flag <<std::endl;
  if (fgo_marginal_flag == MARGIN_SECOND_NEW ||
      d_gnss_frames.back()->is_zupt_frame) {

    SlidingWindowSecondNew();
    std::cout << "SlidingWindowSecondNew break" << std::endl;
    exit(-1);
  } else {
    // SlidingWindowOld();
    d_gnss_frames.pop_front();
  }
}

void FGO_GpsIMU_SOLVER::SolveFGOCeres()
{
  data2double();
  ceres::Problem problem;
  // ceres::LossFunction *loss_function = new
  // ceres::HuberLoss(cv_huber_loss_parameter);
  ceres::LocalParameterization* local_para_se3 =
    new LocalParameterizationSE3();

  for (int i = 0, n = d_gnss_frames.size(); i < n; ++i) {
    std::cout << "===SolveFGOCeres===" << d_gnss_frames.size() << std::endl;
    problem.AddParameterBlock(para_pose + i * 7, 7, local_para_se3);

    if (d_gnss_frames[i]->have_gnss_info) {
      auto factor = new GnssCeresFactor(
        d_gnss_frames[i]->station2cur_blh_local_data, tmp_lever_arm,
        d_gnss_frames[i]->gnss_llh_variance);
      problem.AddResidualBlock(factor, NULL, para_pose + i * 7);
      std::cout << "Vsolver GNSS time=" << d_gnss_frames[i]->timestamp
                << std::endl;
      std::cout << "Vsolver GNSS value="
                << d_gnss_frames[i]->station2cur_blh_local_data << std::endl;
    }

    if (i != 0) {
      auto factor = new CERES_BULL_IMUFactor(d_gnss_frames[i]->imupreinte, Gw);
      std::cout << "Vsolver IMU_Connect time=" << d_gnss_frames[i]->timestamp
                << std::endl;
      std::cout << "vsolver delta_state_.p="
                << d_gnss_frames[i]->imupreinte->delta_p << std::endl;
      std::cout << "vsolver delta_state_.v="
                << d_gnss_frames[i]->imupreinte->delta_v << std::endl;
      std::cout
        << "vsolver delta_state_.q="
        << d_gnss_frames[i]->imupreinte->delta_q.unit_quaternion().coeffs()
        << std::endl;
      std::cout << "vsolver current_state_.p=" << d_gnss_frames[i]->p_wb
                << std::endl;
      std::cout << "vsolver current_state_.v=" << d_gnss_frames[i]->v_wb
                << std::endl;
      std::cout << "vsolver current_state_.q="
                << d_gnss_frames[i]->q_wb.unit_quaternion().coeffs()
                << std::endl;
      std::cout << "vsolver current_state_bg=" << d_gnss_frames[i]->ba
                << std::endl;
      std::cout << "vsolver current_state_bg=" << d_gnss_frames[i]->bg
                << std::endl;
      std::cout << "vsolver preintegrationlist sumT="
                << d_gnss_frames[i]->imupreinte->sum_dt << std::endl;
      problem.AddResidualBlock(factor, NULL, para_pose + (i - 1) * 7,
                               para_speed + (i - 1) * 3,
                               para_bias + (i - 1) * 6, para_pose + i * 7,
                               para_speed + i * 3, para_bias + i * 6);

      // double **para = new double *[5];
      // para[0] = para_pose + (i - 1) * 7;
      // para[1] = para_speed_bias + (i - 1) * 9;
      // para[2] = para_pose + i * 7;
      // para[3] = para_speed_bias + i * 9;
      // factor->check(para);
    }
  }

  {
    auto factor = new CERES_LAST_IMU_BIAS_ERROR();
    problem.AddResidualBlock(factor, NULL,
                             para_bias + (d_gnss_frames.size() - 1) * 6);
  }

  // 边缘化残差
  // prior factor
  if (last_margin_info && last_margin_info->isValid()) {
    auto factor = new MarginalizationFactorSE3Order(last_margin_info);
    problem.AddResidualBlock(factor, nullptr,
                             last_marginalization_info_para_block);
  }

  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_SCHUR;
  options.trust_region_strategy_type = ceres::DOGLEG;
  options.max_num_iterations = 4;
  options.num_threads = 1;
  options.max_solver_time_in_seconds = 0.5; // 50 ms for solver and 50 ms
  // for other
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  double2data();
}

void FGO_GpsIMU_SOLVER::SolveBATemplate()
{
  data2double();

  std::cout << "\nStep 1: ProblemInit" << std::endl;
  GraphOptimizor::Problem<Scalar> problem;
  std::vector<std::shared_ptr<GraphOptimizor::BAVertexSE3Pose<Scalar>>>
    vertexCams_vec;
  std::vector<std::shared_ptr<GraphOptimizor::BAVertexSpeed<Scalar>>>
    vertexV_vec;
  std::vector<std::shared_ptr<GraphOptimizor::BAVertexBias<Scalar>>>
    vertexBias_vec;
  size_t type_cameraVertex = 0;
  size_t type_landmarkVertex = 1;
  int pose_dim = 0;

  std::cout << "\nStep 2: SetVertex" << std::endl;
  for (int i = 0, n = d_gnss_frames.size(); i < n; ++i) {
    Eigen::Map<Eigen::Matrix<Scalar, 7, 1>> pose(para_pose + i * 7);
    std::shared_ptr<GraphOptimizor::BAVertexSE3Pose<Scalar>> vertexCam =
      std::make_shared<GraphOptimizor::BAVertexSE3Pose<Scalar>>();
    vertexCam->SetParameters(pose);
    vertexCam->SetType(type_cameraVertex);
    vertexCams_vec.emplace_back(vertexCam);
    problem.AddVertex(vertexCam);
    pose_dim += vertexCam->GetCalculationDimension();

    std::shared_ptr<GraphOptimizor::BAVertexSpeed<Scalar>> vertexVelocity =
      std::make_shared<GraphOptimizor::BAVertexSpeed<Scalar>>();
    Eigen::Map<Eigen::Matrix<Scalar, 3, 1>> vel_body(para_speed + i * 3);
    vertexVelocity->SetParameters(vel_body);
    vertexVelocity->SetType(type_cameraVertex);
    vertexV_vec.emplace_back(vertexVelocity);
    problem.AddVertex(vertexVelocity);
    pose_dim += vertexVelocity->GetCalculationDimension();

    std::shared_ptr<GraphOptimizor::BAVertexBias<Scalar>> vertexBiasAG =
      std::make_shared<GraphOptimizor::BAVertexBias<Scalar>>();
    Eigen::Map<Eigen::Matrix<Scalar, 6, 1>> bias_body_ag(para_bias + i * 6);
    vertexBiasAG->SetParameters(bias_body_ag);
    vertexBiasAG->SetType(type_cameraVertex);
    vertexBias_vec.emplace_back(vertexBiasAG);
    problem.AddVertex(vertexBiasAG);
    pose_dim += vertexBiasAG->GetCalculationDimension();
  }

  std::cout << "\nStep 2: SetEdge" << std::endl;
  for (int i = 0, n = d_gnss_frames.size(); i < n; ++i) {
    if (d_gnss_frames[i]->have_gnss_info) {
      std::cout << "d_gnss_frames[i]->station2cur_blh_local_data="
                << d_gnss_frames[i]->station2cur_blh_local_data << std::endl;
      std::cout << "d_gnss_frames[i]->gnss_llh_variance="
                << d_gnss_frames[i]->gnss_llh_variance << std::endl;
                
      std::shared_ptr<GraphOptimizor::EdgeGNSS<Scalar>> gnss_edge =
        std::make_shared<GraphOptimizor::EdgeGNSS<Scalar>>(
          d_gnss_frames[i]->station2cur_blh_local_data, tmp_lever_arm,
          d_gnss_frames[i]->gnss_llh_variance);
      gnss_edge->AddVertex(vertexCams_vec[i], 0);
      // std::shared_ptr<GraphOptimizor::HuberKernel<Scalar>> kernel(new
      // GraphOptimizor::HuberKernel<Scalar>(1.0)); edge->SetKernel(kernel);
      problem.AddEdge(gnss_edge);
    }

    if (i == 0)
      continue;

    {
      // imu edge
      std::shared_ptr<GraphOptimizor::EdgeIMU_Preintegration<Scalar>> imuEdge =
        std::make_shared<GraphOptimizor::EdgeIMU_Preintegration<Scalar>>(
          d_gnss_frames[i]->imupreinte, Gw);
      imuEdge->AddVertex(vertexCams_vec[i - 1], 0);
      imuEdge->AddVertex(vertexV_vec[i - 1], 1);
      imuEdge->AddVertex(vertexBias_vec[i - 1], 2);
      imuEdge->AddVertex(vertexCams_vec[i], 3);
      imuEdge->AddVertex(vertexV_vec[i], 4);
      imuEdge->AddVertex(vertexBias_vec[i], 5);
      problem.AddEdge(imuEdge);
    }
  }

  {
    std::shared_ptr<GraphOptimizor::EdgeBiasLast<Scalar>> edge_last_bias =
      std::make_shared<GraphOptimizor::EdgeBiasLast<Scalar>>();
    edge_last_bias->AddVertex(vertexBias_vec.back(), 0);
    problem.AddEdge(edge_last_bias);
  }

  // 已经有 Prior 了
  if (prior_H.rows() > 0) {
    // 外参数先验设置为 0. TODO:: 这个应该放到 solver 里去弄
    // std::cout << "\033[1;31mbold --- prior_H.rows() :: --- "
    //              "\033[0m"
    //           << prior_H.rows() << std::endl;
    problem.SetPrior(prior_H, prior_b, prior_JTinv, prior_r);
  }

  problem.SetMargnedVertexTypesWhenSolving(
    type_landmarkVertex); // 设置在求解过程中需要暂时被边缘化的节点的类型

  /*Solver_Settings*/
#if 0
  // 设置数值优化方法
  problem.SetMethod(GraphOptimizor::Problem<Scalar>::Method::LM_Auto); //
  // problem.SetLinearSolver(GraphOptimizor::Problem<Scalar>::LinearSolver::LDLT_Solver);
  problem.SetLinearSolver(GraphOptimizor::Problem<Scalar>::LinearSolver::PCG_Solver);
  // problem.SetMethod(GraphOptimizor::Problem<Scalar>::Method::DogLeg);
  GraphOptimizor::Problem<Scalar>::Options options; // 使用默认参数

  // 算法的阻尼因子调整参数
  // problem.LM_SetDampParameter(11.0, 10.0);    // 设置 LM

  // std::cout << "options.maxInvalidStep = " << options.maxInvalidStep
  //           << std::endl;
  // std::cout << "options.minNormDeltaX = " << options.minNormDeltaX
  //           << std::endl;
#else
  // if (prior_H.rows() > 0) {
    problem.SetMethod(GraphOptimizor::Problem<Scalar>::Method::DogLeg);
    problem.SetLinearSolver(
      GraphOptimizor::Problem<Scalar>::LinearSolver::LDLT_Solver);
  // } else {
  //   problem.SetMethod(GraphOptimizor::Problem<Scalar>::Method::DogLeg);
  //   problem.SetLinearSolver(
  //     GraphOptimizor::Problem<Scalar>::LinearSolver::PCG_Solver);
  // }
  GraphOptimizor::Problem<Scalar>::Options options; // 使用默认参数
#endif

#if 1
  options.maxInvalidStep = 1;
  options.maxLambda = 1e20;
  options.maxMinCostHold = 1;
  options.maxRadius = 20;
  options.minCostDownRate = 1e-6;
  options.minLambda = 1e-8;
  options.minNormDeltaX = 1e-5;
  options.minPCGCostDownRate = 1e-6;
  options.minRadius = 1e-4;
  options.maxTimeCost = 20;
#endif

  problem.SetOptions(options);
  problem.Solve(4);

  // update P Q V ba bg vertex
  for (int i = 0, n = d_gnss_frames.size(); i < n; ++i) {
    GraphOptimizor::Vector7<Scalar> p = vertexCams_vec[i]->GetParameters();

    for (int j = 0; j < 7; ++j) {
      para_pose[(i * 7 + j)] = p[j];
    }

    GraphOptimizor::Vector3<Scalar> v = vertexV_vec[i]->GetParameters();
    for (int j = 0; j < 3; ++j) {
      para_speed[i * 3 + j] = v[j];
    }

    GraphOptimizor::Vector6<Scalar> bias_ag =
      vertexBias_vec[i]->GetParameters();
    for (int j = 0; j < 6; ++j) {
      para_bias[i * 6 + j] = bias_ag[j];
    }
  }

  double2data();
}

void FGO_GpsIMU_SOLVER::MarginalizeBATemplate()
{
  if (fgo_marginal_flag == MARGIN_OLD) {
    data2double();

    std::cout << "\nStep 1: ProblemInit" << std::endl;
    GraphOptimizor::Problem<Scalar> problem;
    std::vector<std::shared_ptr<GraphOptimizor::BAVertexSE3Pose<Scalar>>>
      vertexCams_vec;
    std::vector<std::shared_ptr<GraphOptimizor::BAVertexSpeed<Scalar>>>
      vertexV_vec;
    std::vector<std::shared_ptr<GraphOptimizor::BAVertexBias<Scalar>>>
      vertexBias_vec;
    size_t type_cameraVertex = 0;
    size_t type_landmarkVertex = 1;
    int pose_dim = 0;

    std::cout << "\nStep 2: SetVertex" << std::endl;
    for (int i = 0, n = d_gnss_frames.size(); i < n; ++i) {
      Eigen::Map<Eigen::Matrix<Scalar, 7, 1>> pose(para_pose + i * 7);
      std::shared_ptr<GraphOptimizor::BAVertexSE3Pose<Scalar>> vertexCam =
        std::make_shared<GraphOptimizor::BAVertexSE3Pose<Scalar>>();
      vertexCam->SetParameters(pose);
      vertexCam->SetType(type_cameraVertex);
      vertexCams_vec.emplace_back(vertexCam);
      problem.AddVertex(vertexCam);
      pose_dim += vertexCam->GetCalculationDimension();

      std::shared_ptr<GraphOptimizor::BAVertexSpeed<Scalar>> vertexVelocity =
        std::make_shared<GraphOptimizor::BAVertexSpeed<Scalar>>();
      Eigen::Map<Eigen::Matrix<Scalar, 3, 1>> vel_body(para_speed + i * 3);
      vertexVelocity->SetParameters(vel_body);
      vertexVelocity->SetType(type_cameraVertex);
      vertexV_vec.emplace_back(vertexVelocity);
      problem.AddVertex(vertexVelocity);
      pose_dim += vertexVelocity->GetCalculationDimension();

      std::shared_ptr<GraphOptimizor::BAVertexBias<Scalar>> vertexBiasAG =
        std::make_shared<GraphOptimizor::BAVertexBias<Scalar>>();
      Eigen::Map<Eigen::Matrix<Scalar, 6, 1>> bias_body_ag(para_bias + i * 6);
      vertexBiasAG->SetParameters(bias_body_ag);
      vertexBiasAG->SetType(type_cameraVertex);
      vertexBias_vec.emplace_back(vertexBiasAG);
      problem.AddVertex(vertexBiasAG);
      pose_dim += vertexBiasAG->GetCalculationDimension();
    }

    std::cout << "\nStep 2: SetEdge" << std::endl;
    for (int i = 0, n = d_gnss_frames.size(); i < n; ++i) {
      if (d_gnss_frames[i]->have_gnss_info) {
        std::shared_ptr<GraphOptimizor::EdgeGNSS<Scalar>> gnss_edge =
          std::make_shared<GraphOptimizor::EdgeGNSS<Scalar>>(
            d_gnss_frames[i]->station2cur_blh_local_data, tmp_lever_arm,
            d_gnss_frames[i]->gnss_llh_variance);
        gnss_edge->AddVertex(vertexCams_vec[i], 0);
        problem.AddEdge(gnss_edge);
      }

      if (i == 0)
        continue;

      {
        // imu edge
        std::shared_ptr<GraphOptimizor::EdgeIMU_Preintegration<Scalar>>
          imuEdge =
            std::make_shared<GraphOptimizor::EdgeIMU_Preintegration<Scalar>>(
              d_gnss_frames[i]->imupreinte, Gw);
        imuEdge->AddVertex(vertexCams_vec[i - 1], 0);
        imuEdge->AddVertex(vertexV_vec[i - 1], 1);
        imuEdge->AddVertex(vertexBias_vec[i - 1], 2);
        imuEdge->AddVertex(vertexCams_vec[i], 3);
        imuEdge->AddVertex(vertexV_vec[i], 4);
        imuEdge->AddVertex(vertexBias_vec[i], 5);
        problem.AddEdge(imuEdge);
      }
    }

    // 已经有 Prior 了
    if (prior_H.rows() > 0) {
      // 外参数先验设置为 0. TODO:: 这个应该放到 solver 里去弄
      std::cout << "\033[1;31mbold --- prior_H.rows() :: --- "
                   "\033[0m"
                << prior_H.rows() << std::endl;
      problem.SetPrior(prior_H, prior_b, prior_JTinv, prior_r);
    }

    // todo :: add margin term
    problem.SetMargnedVertexTypesWhenSolving(
      type_landmarkVertex); // 设置在求解过程中需要暂时被边缘化的节点的类型
    std::vector<std::shared_ptr<GraphOptimizor::VertexBase<Scalar>>> needMarg;
    int margin_vertex_size = 0;
    needMarg.emplace_back(vertexCams_vec.front());
    margin_vertex_size += vertexCams_vec.front()->GetCalculationDimension();
    needMarg.emplace_back(vertexV_vec.front());
    margin_vertex_size += vertexV_vec.front()->GetCalculationDimension();
    needMarg.emplace_back(vertexBias_vec.front());
    margin_vertex_size += vertexBias_vec.front()->GetCalculationDimension();

    problem.Marginalize(needMarg, pose_dim - margin_vertex_size);
    problem.GetPrior(prior_H, prior_b, prior_JTinv, prior_r);

  } // MARGIN_OLD Finish.
  else if (Hprior_.rows() > 0) {
    std::cout
      << "\033[1;31mbold ---MarginalizeBATemplate:MARGIN_NEW_NotSupport--- "
         "\033[0m\n"
      << std::endl;
    exit(-1);
  }
}

void FGO_GpsIMU_SOLVER::MarginalizeCeres()
{
  if (fgo_marginal_flag == MARGIN_OLD) {
    data2double();
    // auto loss_function = new ceres::HuberLoss(cv_huber_loss_parameter);
    std::shared_ptr<MarginalizationInfo> margin_info =
      std::make_shared<MarginalizationInfo>();
    if (last_margin_info && last_margin_info->isValid()) {
      std::vector<int> drop_set;
      for (int i = 0, n = last_marginalization_info_para_block.size(); i < n;
           ++i) {
        if (last_marginalization_info_para_block[i] == para_pose ||
            last_marginalization_info_para_block[i] == para_speed ||
            last_marginalization_info_para_block[i] == para_bias)
          drop_set.emplace_back(i);
      }
      auto factor =
        std::make_shared<MarginalizationFactorSE3Order>(last_margin_info);
      auto residual = std::make_shared<ResidualBlockInfo>(
        factor, nullptr, last_marginalization_info_para_block, drop_set);
      margin_info->addResidualBlockInfo(residual);
    }

    // IMU残差
    // preintegration factors
    {
      // std::cout << "ba use magin5 preintegration factors" <<std::endl;
      auto factor = std::make_shared<CERES_BULL_IMUFactor>(
        d_gnss_frames[1]->imupreinte, Gw);
      auto residual_imu_preintegration = std::make_shared<ResidualBlockInfo>(
        factor, nullptr,
        std::vector<double*>{ para_pose, para_speed, para_bias, para_pose + 7,
                              para_speed + 3, para_bias + 6 },
        std::vector<int>{ 0, 1, 2 });
      margin_info->addResidualBlockInfo(residual_imu_preintegration);
    }

    // GNSS残差
    // GNSS factors
    if (d_gnss_frames[0]->have_gnss_info) {
      // std::cout << "ba use magin6 gnss factors" <<std::endl;
      auto factor = std::make_shared<GnssCeresFactor>(
        d_gnss_frames[0]->station2cur_blh_local_data, tmp_lever_arm,
        d_gnss_frames[0]->gnss_llh_variance);
      auto gnss_residual = std::make_shared<ResidualBlockInfo>(
        factor, nullptr, std::vector<double*>{ para_pose },
        std::vector<int>{});
      margin_info->addResidualBlockInfo(gnss_residual);
    }

    // std::cout << "ba use magin7" <<std::endl;
    // 边缘化处理
    // do marginalization
    margin_info->marginalization();

    // std::cout << "ba use magin8" <<std::endl;
    std::unordered_map<long, double*> addr_shift;
    for (int i = 1, n = d_gnss_frames.size(); i < n; ++i) {
      addr_shift[reinterpret_cast<long>(para_pose + 7 * i)] =
        para_pose + 7 * (i - 1);
      addr_shift[reinterpret_cast<long>(para_speed + 3 * i)] =
        para_speed + 3 * (i - 1);
      addr_shift[reinterpret_cast<long>(para_bias + 6 * i)] =
        para_bias + 6 * (i - 1);
    }
    last_marginalization_info_para_block =
      margin_info->getParamterBlocks(addr_shift);
    last_margin_info = std::move(margin_info);
    // std::cout << "ba use magin END" <<std::endl;
  } // if (fgo_marginal_flag == MARGIN_OLD)
  else {
    std::cout << " not support ceres MarginalizeCeres" << std::endl;
    exit(-1);
  }
}

void FGO_GpsIMU_SOLVER::SlidingWindowSecondNew()
{
  //*TODOPOI:: estimate_camera_intrinsic == true, pt_n_per_frame need
  // re-estimate through image plane and current cam para
  // vsolver::ScopedTrace trace("SlidingWindowSecondNew");
  // [ 0, 1, ..., size-2, size-1]
  //  kf kf       second     new
  //              XXXXXX
  int size_frames = d_gnss_frames.size();

  GNSSFramePtr second_new = *(d_gnss_frames.end() - 2);
  GNSSFramePtr latest_new = *(d_gnss_frames.end() - 1);

  if (latest_new->is_zupt_frame) {
    // the newest frame is zupt state
    //**NOTE : 1. not implement second_new->imupreinte->push_back to change
    // imu_preintegration
    //**NOTE : 2  if this frame is zupt_frame, not implement repropagate
    //**NOTE : 3  由於下一次的預積分會參考上次的frame最後的v_imu_timestamp,
    // v_gyr, v_acc, 為此我們還是需要加入訊息進去（並且保持imu資料完整）
    latest_new->imupreinte = second_new->imupreinte;
    // only save imu raw data, not push_back to imu_preintegration
    latest_new->v_imu_timestamp.insert(latest_new->v_imu_timestamp.begin(),
                                       second_new->v_imu_timestamp.begin(),
                                       second_new->v_imu_timestamp.end());

    //**NOTE : second_new_begin -> second_new_end -> latest_new.begin ->
    // latest_new_end
    latest_new->v_gyr.insert(latest_new->v_gyr.begin(),
                             second_new->v_gyr.begin(),
                             second_new->v_gyr.end());
    latest_new->v_acc.insert(latest_new->v_acc.begin(),
                             second_new->v_acc.begin(),
                             second_new->v_acc.end());

    // save last gnss info as start_anchor info for this frame.
    latest_new->station2cur_blh_local_data =
      second_new->station2cur_blh_local_data;
    latest_new->ori_blh = second_new->ori_blh;
    latest_new->have_gnss_info = true;
  } else {
    std::cout << "margin new" << std::endl;
    // add last imu info to cur frame
    double t0 = second_new->v_imu_timestamp.back();
    for (int i = 0, n = latest_new->v_acc.size(); i < n; ++i) {
      double dt = latest_new->v_imu_timestamp[i] - t0;
      t0 = latest_new->v_imu_timestamp[i];
      Eigen::Vector3d gyr = latest_new->v_gyr[i], acc = latest_new->v_acc[i];
      second_new->imupreinte->push_back(dt, acc, gyr);
    }
    latest_new->imupreinte = second_new->imupreinte;
    latest_new->v_imu_timestamp.insert(latest_new->v_imu_timestamp.begin(),
                                       second_new->v_imu_timestamp.begin(),
                                       second_new->v_imu_timestamp.end());
    latest_new->v_gyr.insert(latest_new->v_gyr.begin(),
                             second_new->v_gyr.begin(),
                             second_new->v_gyr.end());
    latest_new->v_acc.insert(latest_new->v_acc.begin(),
                             second_new->v_acc.begin(),
                             second_new->v_acc.end());

    // save last gnss info as start_anchor info for this frame.
    latest_new->station2cur_blh_local_data =
      second_new->station2cur_blh_local_data;
    latest_new->ori_blh = second_new->ori_blh;
    latest_new->have_gnss_info = true;
  }

  // final step
  d_gnss_frames.erase(d_gnss_frames.end() - 2);
  std::cout << "latest_new->imupreinte  imu size="
            << latest_new->imupreinte->acc_buf.size() << std::endl;
}

void FGO_GpsIMU_SOLVER::Reset()
{
  // Rest InitValue
  state = Gw_NEED_INIT;
  d_gnss_frames.clear();
  initGPS = false;
  Hprior_ = MatXX(0, 0);
  Jprior_inv_ = MatXX(0, 0);
  Jprior_ = MatXX(0, 0);
  bprior_ = VecX(0);
  errprior_ = VecX(0);
  gnss_system_initial_timestamp = 0;
  zupt_acc_norm.clear();
  auto_detect_imu_fps = -1;
  station_origin_llh.setZero();

  // ceres only
  last_marginalization_info_para_block.clear();
  last_margin_info = nullptr;
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
}
