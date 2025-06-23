// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2021 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Jason Zhou
// =============================================================================
//
// Demo to show Viper Rover operated on SCM Terrain
//
// =============================================================================

#include "chrono_models/robot/viper/Viper.h"

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/terrain/SCMTerrain.h"

#include "chrono_thirdparty/filesystem/path.h"

#ifdef CHRONO_POSTPROCESS
    #include "chrono_postprocess/ChGnuPlot.h"
#endif

#include "chrono/assets/ChVisualSystem.h"
#ifdef CHRONO_IRRLICHT
    #include "chrono_irrlicht/ChVisualSystemIrrlicht.h"
using namespace chrono::irrlicht;
#endif
#ifdef CHRONO_VSG
    #include "chrono_vsg/ChVisualSystemVSG.h"
using namespace chrono::vsg3d;
#endif


//! Custom File 생성
#include "data_logging.hpp"
#include "rover.hpp"
#include "motionctrl.hpp"

//! Class 선언
DataLogging<double> data_logger;
Mclrover<double> rover;
motionctrl<double> motion_ctrl;

//! 스마트 포인터 사용
std::shared_ptr<Mclrover<double>::model> model_ptr;
std::shared_ptr<Mclrover<double>::Traj> traj_ptr;
std::shared_ptr<Mclrover<double>::ctrl> ctrl_ptr;

using namespace chrono;
using namespace chrono::viper;

// -----------------------------------------------------------------------------

// Run-time visualization system (IRRLICHT or VSG)
ChVisualSystem::Type vis_type = ChVisualSystem::Type::VSG;

bool output = false;

// SCM grid spacing
double mesh_resolution = 0.01;


// Enable/disable bulldozing effects
bool enable_bulldozing = true;

// Enable/disable moving patch feature
bool enable_moving_patch = true;

// If true, use provided callback to change soil properties based on location
bool var_params = true;

// Define Viper rover wheel type
ViperWheelType wheel_type = ViperWheelType::RealWheel;

// -----------------------------------------------------------------------------

// Custom callback for setting location-dependent soil properties.
// Note that the location is given in the SCM reference frame.
class MySoilParams : public vehicle::SCMTerrain::SoilParametersCallback {
  public:
    virtual void Set(const ChVector3d& loc,
                     double& Bekker_Kphi,
                     double& Bekker_Kc,
                     double& Bekker_n,
                     double& Mohr_cohesion,
                     double& Mohr_friction,
                     double& Janosi_shear,
                     double& elastic_K,
                     double& damping_R) override {
        Bekker_Kphi = 814000; //0.82e6
        Bekker_Kc = 1370; //0.14e4
        Bekker_n = 1.0;
        Mohr_cohesion = 800; //0.017e4
        Mohr_friction = 37.2;
        Janosi_shear = 0.0383; //단위가 센티미터? or 미터 1.78e-2
        elastic_K = 2e8;
        damping_R = 3e4;
    }
};

// Use custom material for the Viper Wheel
bool use_custom_mat = true;

// -----------------------------------------------------------------------------

// Return customized wheel material parameters
std::shared_ptr<ChContactMaterial> CustomWheelMaterial(ChContactMethod contact_method) {
    float mu = 0.4f;   // coefficient of friction
    float cr = 0.1f;   // coefficient of restitution
    float Y = 2e7f;    // Young's modulus
    float nu = 0.3f;   // Poisson ratio
    float kn = 2e5f;   // normal stiffness
    float gn = 40.0f;  // normal viscous damping
    float kt = 2e5f;   // tangential stiffness
    float gt = 20.0f;  // tangential viscous damping

    switch (contact_method) {
        case ChContactMethod::NSC: {
            auto matNSC = chrono_types::make_shared<ChContactMaterialNSC>();
            matNSC->SetFriction(mu);
            matNSC->SetRestitution(cr);
            return matNSC;
        }
        case ChContactMethod::SMC: {
            auto matSMC = chrono_types::make_shared<ChContactMaterialSMC>();
            matSMC->SetFriction(mu);
            matSMC->SetRestitution(cr);
            matSMC->SetYoungModulus(Y);
            matSMC->SetPoissonRatio(nu);
            matSMC->SetKn(kn);
            matSMC->SetGn(gn);
            matSMC->SetKt(kt);
            matSMC->SetGt(gt);
            return matSMC;
        }
        default:
            return std::shared_ptr<ChContactMaterial>();
    }
}

// -----------------------------------------------------------------------------

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    // Global parameter for moving patch size:
    double wheel_range = 0.5;
    ////double body_range = 1.2;

    //! ptr 만들어서 포인터 주소 set 해주는 곳
    model_ptr = rover.set_model_ptr();
    traj_ptr = rover.set_traj_ptr();
    ctrl_ptr = rover.set_ctrl_ptr();


    //! 다른 Folder로 포인터 넘겨주는 곳
    data_logger.get_Mclrovoer_ptr(model_ptr, traj_ptr, ctrl_ptr);
    motion_ctrl.get_Mclrovoer_ptr(model_ptr, traj_ptr, ctrl_ptr);


    // Create a Chrono physical system and associated collision system
    ChSystemSMC sys;
    sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
    // sys.SetGravitationalAcceleration(ChVector3d(0, 0, -1.67));
    sys.SetGravitationalAcceleration(ChVector3d(0, 0, -3));

    // Initialize output
    const std::string out_dir = GetChronoOutputPath() + "SCM_DEF_SOIL";
    if (output) {
        if (!filesystem::create_directory(filesystem::path(out_dir))) {
            std::cout << "Error creating directory " << out_dir << std::endl;
            return 1;
        }
    }
    utils::ChWriterCSV csv(" ");

    // Create the rover
    auto driver = chrono_types::make_shared<ViperDCMotorControl>(); //

    Viper viper(&sys, wheel_type); //* Viper 객체 생성

    viper.SetDriver(driver);

    if (use_custom_mat)
        viper.SetWheelContactMaterial(CustomWheelMaterial(ChContactMethod::SMC));

    double Z_rotation_angle_degrees = 45.0;
    ChQuaternion<> initial_rotation = QuatFromAngleZ(Z_rotation_angle_degrees * CH_DEG_TO_RAD);


    // viper.Initialize(ChFrame<>(ChVector3d(-1, 0, 0.001), initial_rotation));
    viper.Initialize(ChFrame<>(ChVector3d(-1, 0, 0.001), QUNIT));

    // Get wheels and bodies to set up SCM patches
    auto Wheel_1 = viper.GetWheel(ViperWheelID::V_LF)->GetBody();
    auto Wheel_2 = viper.GetWheel(ViperWheelID::V_RF)->GetBody();
    auto Wheel_3 = viper.GetWheel(ViperWheelID::V_LB)->GetBody();
    auto Wheel_4 = viper.GetWheel(ViperWheelID::V_RB)->GetBody();
    auto Body_1 = viper.GetChassis()->GetBody();


    // model_ptr->body_pos_ = Eigen::Vector3d(0,0,0);
    //
    // THE DEFORMABLE TERRAIN
    //

    // Create the 'deformable terrain' object
    vehicle::SCMTerrain terrain(&sys);

    // Displace/rotate the terrain reference plane.
    // Note that SCMTerrain uses a default ISO reference frame (Z up). Since the mechanism is modeled here in
    // a Y-up global frame, we rotate the terrain plane by -90 degrees about the X axis.
    // Note: Irrlicht uses a Y-up frame
    terrain.SetPlane(ChCoordsys<>(ChVector3d(0, 0, -0.25)));

    // 지면 기울이기
    // terrain.SetPlane(ChCoordsys<>(ChVector3d(0, 0, -1.2), ChQuaternion(0.9962, 0.0, 0.0872, 0.0)));

    // Use a regular grid:
    double length = 14;
    double width = 4;
    terrain.Initialize(length, width, mesh_resolution); //! This Part Choose Other callback function for height map

    // Set the soil terramechanical parameters
    if (var_params) {
        // Here we use the soil callback defined at the beginning of the code
        auto my_params = chrono_types::make_shared<MySoilParams>();
        terrain.RegisterSoilParametersCallback(my_params);
    } else {
        // If var_params is set to be false, these parameters will be used
        //! Terrain SetSoil Parameter는 위에서 설정
        terrain.SetSoilParameters(0.2e6,  // Bekker Kphi
                                  0,      // Bekker Kc
                                  1.1,    // Bekker n exponent
                                  0,      // Mohr cohesive limit (Pa)
                                  30,     // Mohr friction limit (degrees)
                                  0.01,   // Janosi shear coefficient (m)
                                  4e7,    // Elastic stiffness (Pa/m), before plastic yield, must be > Kphi
                                  3e4     // Damping (Pa s/m), proportional to negative vertical speed (optional)
        );
    }

    // Set up bulldozing factors
    if (enable_bulldozing) {
        terrain.EnableBulldozing(true);  // inflate soil at the border of the rut
        terrain.SetBulldozingParameters(
            55,  // angle of friction for erosion of displaced material at the border of the rut
            1,   // displaced material vs downward pressed material.
            5,   // number of erosion refinements per timestep
            6);  // number of concentric vertex selections subject to erosion
    }

    // We need to add a moving patch under every wheel
    // Or we can define a large moving patch at the pos of the rover body
    //! 바퀴 주변에만 충돌 계산을 하도록 위치를 정할 수 있음
    if (enable_moving_patch) {
        terrain.AddMovingPatch(Wheel_1, ChVector3d(0, 0, 0), ChVector3d(0.5, 2 * wheel_range, 2 * wheel_range));
        terrain.AddMovingPatch(Wheel_2, ChVector3d(0, 0, 0), ChVector3d(0.5, 2 * wheel_range, 2 * wheel_range));
        terrain.AddMovingPatch(Wheel_3, ChVector3d(0, 0, 0), ChVector3d(0.5, 2 * wheel_range, 2 * wheel_range));
        terrain.AddMovingPatch(Wheel_4, ChVector3d(0, 0, 0), ChVector3d(0.5, 2 * wheel_range, 2 * wheel_range));
    }

    // Set some visualization parameters: either with a texture, or with falsecolor plot, etc.
    terrain.SetPlotType(vehicle::SCMTerrain::PLOT_PRESSURE, 0, 20000);

    terrain.SetMeshWireframe(true);

    // Create the run-time visualization interface
#ifndef CHRONO_IRRLICHT
    if (vis_type == ChVisualSystem::Type::IRRLICHT)
        vis_type = ChVisualSystem::Type::VSG;
#endif
#ifndef CHRONO_VSG
    if (vis_type == ChVisualSystem::Type::VSG)
        vis_type = ChVisualSystem::Type::IRRLICHT;
#endif

    std::shared_ptr<ChVisualSystem> vis;
    switch (vis_type) {
        case ChVisualSystem::Type::IRRLICHT: {
#ifdef CHRONO_IRRLICHT
            auto vis_irr = chrono_types::make_shared<ChVisualSystemIrrlicht>();
            vis_irr->AttachSystem(&sys);
            vis_irr->SetCameraVertical(CameraVerticalDir::Z);
            vis_irr->SetWindowSize(800, 600);
            vis_irr->SetWindowTitle("Viper Rover on SCM");
            vis_irr->Initialize();
            vis_irr->AddLogo();
            vis_irr->AddSkyBox();
            vis_irr->AddCamera(ChVector3d(1.0, 2.0, 1.4), ChVector3d(0, 0, wheel_range));
            vis_irr->AddTypicalLights();
            vis_irr->AddLightWithShadow(ChVector3d(-5.0, -0.5, 8.0), ChVector3d(-1, 0, 0), 100, 1, 35, 85, 512,
                                        ChColor(0.8f, 0.8f, 0.8f));
            vis_irr->EnableShadows();

            vis = vis_irr;
#endif
            break;
        }
        default:
        case ChVisualSystem::Type::VSG: {
#ifdef CHRONO_VSG
            auto vis_vsg = chrono_types::make_shared<ChVisualSystemVSG>();
            vis_vsg->AttachSystem(&sys);
            vis_vsg->SetWindowSize(800, 600);
            vis_vsg->SetWindowTitle("Viper Rover on SCM");
            vis_vsg->AddCamera(ChVector3d(1.0, 2.0, 1.4), ChVector3d(0, 0, wheel_range));
            vis_vsg->Initialize();

            vis = vis_vsg;
#endif
            break;
        }
    }





    //! Realtime Loop 도는 곳
    while (vis->Run()) {
#if defined(CHRONO_IRRLICHT) || defined(CHRONO_VSG)

        //!Sensor Data 측정
        //! Sensor Data를 사용하는 부분. 이 부분은 추후에 함수로 빼야함

        double Drive_Gear_ratio = 1; //! Sensor는 휠단을 받아온다
        double Steer_Gear_ratio = 1;
        double Sus_Gear_ratio = 400;

        auto chassisPos = viper.GetChassisPos();
        auto chassisRot = viper.GetChassisRot();
        auto chassisVel_global = viper.GetChassisVel();
        auto chassisAngVel_global = viper.GetChassisRotVel();
        auto chassisAcc_global = viper.GetChassisAcc();
        auto chassisAngAcc_global = viper.GetChassisRotAcc();
        auto FLwheel_vel = viper.FLWheelvel(); // lift, steer, drive 순서
        auto FRwheel_vel = viper.FRWheelvel();
        auto RLwheel_vel = viper.RLWheelvel();
        auto RRwheel_vel = viper.RRWheelvel();
        auto FLwheel_pos = viper.FLWheelpos();
        auto FRwheel_pos = viper.FRWheelpos();
        auto RLwheel_pos = viper.RLWheelpos();
        auto RRwheel_pos = viper.RRWheelpos();
        auto FLwheel_grf_global = viper.GetWheelAppliedForce(V_LF);
        auto FRwheel_grf_global = viper.GetWheelAppliedForce(V_RF);
        auto RLwheel_grf_global = viper.GetWheelAppliedForce(V_LB);
        auto RRwheel_grf_global = viper.GetWheelAppliedForce(V_RB);
        auto FLwheel_grt = viper.GetWheelContactTorque(V_LF);
        auto FRwheel_grt = viper.GetWheelContactTorque(V_RF);
        auto RLwheel_grt = viper.GetWheelContactTorque(V_LB);
        auto RRwheel_grt = viper.GetWheelContactTorque(V_RB);

        auto FLwheel_linvel_global = viper.GetWheel(ViperWheelID::V_LF)->GetLinVel();
        auto FRwheel_linvel_global = viper.GetWheel(ViperWheelID::V_RF)->GetLinVel();
        auto RLwheel_linvel_global = viper.GetWheel(ViperWheelID::V_LB)->GetLinVel();
        auto RRwheel_linvel_global = viper.GetWheel(ViperWheelID::V_RB)->GetLinVel();

        auto q_wheel_FL = viper.GetWheel(ViperWheelID::V_LF)->GetRot();
        auto q_wheel_FR = viper.GetWheel(ViperWheelID::V_RF)->GetRot();
        auto q_wheel_RL = viper.GetWheel(ViperWheelID::V_LB)->GetRot();
        auto q_wheel_RR = viper.GetWheel(ViperWheelID::V_RB)->GetRot();

        auto FLwheel_loc = viper.GetWheel(ViperWheelID::V_LF)->GetPos();
        FLwheel_loc[0] = FLwheel_loc[0] + 0;
        auto ter_FL = terrain.GetNodeInfo(FLwheel_loc);

        auto FRwheel_loc = viper.GetWheel(ViperWheelID::V_RF)->GetPos();
        auto ter_FR = terrain.GetNodeInfo(FRwheel_loc);

        auto RLwheel_loc = viper.GetWheel(ViperWheelID::V_LB)->GetPos();
        auto ter_RL = terrain.GetNodeInfo(RLwheel_loc);

        auto RRwheel_loc = viper.GetWheel(ViperWheelID::V_RB)->GetPos();
        auto ter_RR = terrain.GetNodeInfo(RRwheel_loc);

        // Translation for Body Frame
        auto chassisVel = chassisRot.RotateBack(chassisVel_global);
        auto chassisAngVel = chassisRot.RotateBack(chassisAngVel_global);
        auto chassisAcc = chassisRot.RotateBack(chassisAcc_global);
        auto chassisAngAcc = chassisRot.RotateBack(chassisAngAcc_global);

        // ZYX 순서 (Yaw, Pitch, Roll)로 오일러 각 얻기 시도
        ChVector3d euler_ZYX = chassisRot.GetCardanAnglesZYX();
        double roll_rad = euler_ZYX.x();   // Z축 회전 (Yaw)
        double pitch_rad = euler_ZYX.y(); // Y축 회전 (Pitch)
        double yaw_rad = euler_ZYX.z();  // X축 회전 (Roll)

        // auto FLwheel_grf = chassisRot.RotateBack(FLwheel_grf_global);
        // auto FRwheel_grf = chassisRot.RotateBack(FRwheel_grf_global);
        // auto RLwheel_grf = chassisRot.RotateBack(RLwheel_grf_global);
        // auto RRwheel_grf = chassisRot.RotateBack(RRwheel_grf_global);

        // Translation for Wheel Frame

        auto FLwheel_grf = q_wheel_FL.RotateBack(FLwheel_grf_global);
        auto FRwheel_grf = q_wheel_FR.RotateBack(FRwheel_grf_global);
        auto RLwheel_grf = q_wheel_RL.RotateBack(RLwheel_grf_global);
        auto RRwheel_grf = q_wheel_RR.RotateBack(RRwheel_grf_global);

        auto FLwheel_linvel = q_wheel_FL.RotateBack(FLwheel_linvel_global);
        auto FRwheel_linvel = q_wheel_FR.RotateBack(FRwheel_linvel_global);
        auto RLwheel_linvel = q_wheel_RL.RotateBack(RLwheel_linvel_global);
        auto RRwheel_linvel = q_wheel_RR.RotateBack(RRwheel_linvel_global);

        model_ptr->wheel_lin_vel_[0] = Eigen::Vector3d(FLwheel_linvel.x()*cos(FLwheel_pos[2]) + FLwheel_linvel.z()*sin(FLwheel_pos[2]), -FLwheel_linvel.y(), FLwheel_linvel.z());
        model_ptr->wheel_lin_vel_[1] = Eigen::Vector3d(FRwheel_linvel.x()*cos(FRwheel_pos[2]) + FRwheel_linvel.z()*sin(FRwheel_pos[2]), -FRwheel_linvel.y(), FRwheel_linvel.z());
        model_ptr->wheel_lin_vel_[2] = Eigen::Vector3d(RLwheel_linvel.x()*cos(RLwheel_pos[2]) + RLwheel_linvel.z()*sin(RLwheel_pos[2]), -RLwheel_linvel.y(), RLwheel_linvel.z());
        model_ptr->wheel_lin_vel_[3] = Eigen::Vector3d(RRwheel_linvel.x()*cos(RRwheel_pos[2]) + RRwheel_linvel.z()*sin(RRwheel_pos[2]), -RRwheel_linvel.y(), RRwheel_linvel.z());



        model_ptr->body_pos_ = Eigen::Vector3d(chassisPos.x(), chassisPos.y(), chassisPos.z());
        model_ptr->body_ang_pos_ = Eigen::Vector4d(chassisRot.e0(), chassisRot.e1(), chassisRot.e2(), chassisRot.e3());
        model_ptr->body_vel_ = Eigen::Vector3d(chassisVel.x(), chassisVel.y(), chassisVel.z());
        model_ptr->body_ang_vel_ = Eigen::Vector3d(chassisAngVel.x(), chassisAngVel.y(), chassisAngVel.z());
        model_ptr->body_acc_ = Eigen::Vector3d(chassisAcc.x(), chassisAcc.y(), chassisAcc.z());
        model_ptr->body_ang_acc_ = Eigen::Vector3d(chassisAngAcc.x(), chassisAngAcc.y(), chassisAngAcc.z());
        //! Sus 부호신경 써줘야함
        model_ptr->joint_vel_act_[0] = Eigen::Vector3d(-FLwheel_vel[0]/Sus_Gear_ratio, FLwheel_vel[1]/Steer_Gear_ratio, FLwheel_vel[2]/Drive_Gear_ratio);
        model_ptr->joint_vel_act_[1] = Eigen::Vector3d(FRwheel_vel[0]/Sus_Gear_ratio, FRwheel_vel[1]/Steer_Gear_ratio, FRwheel_vel[2]/Drive_Gear_ratio);
        model_ptr->joint_vel_act_[2] = Eigen::Vector3d(-RLwheel_vel[0]/Sus_Gear_ratio, RLwheel_vel[1]/Steer_Gear_ratio, RLwheel_vel[2]/Drive_Gear_ratio);
        model_ptr->joint_vel_act_[3] = Eigen::Vector3d(RRwheel_vel[0]/Sus_Gear_ratio, RRwheel_vel[1]/Steer_Gear_ratio, RRwheel_vel[2]/Drive_Gear_ratio);
        model_ptr->joint_pos_act_[0] = Eigen::Vector3d(-FLwheel_pos[0]/Sus_Gear_ratio, (FLwheel_pos[1])/Steer_Gear_ratio-yaw_rad, FLwheel_pos[2]/Drive_Gear_ratio);
        model_ptr->joint_pos_act_[1] = Eigen::Vector3d(FRwheel_pos[0]/Sus_Gear_ratio, (FRwheel_pos[1])/Steer_Gear_ratio-yaw_rad, FRwheel_pos[2]/Drive_Gear_ratio);
        model_ptr->joint_pos_act_[2] = Eigen::Vector3d(-RLwheel_pos[0]/Sus_Gear_ratio, (RLwheel_pos[1])/Steer_Gear_ratio-yaw_rad, RLwheel_pos[2]/Drive_Gear_ratio);
        model_ptr->joint_pos_act_[3] = Eigen::Vector3d(RRwheel_pos[0]/Sus_Gear_ratio, (RRwheel_pos[1])/Steer_Gear_ratio-yaw_rad, RRwheel_pos[2]/Drive_Gear_ratio);




        //! 확인해야 하는 부분 : GRF 부호, Torque 부호 및 좌표계로 힘 좌표계 생각해야함

        //* Rover Frame GRF
        // model_ptr->contact_force_[0] = Eigen::Vector3d(FLwheel_grf.x(), FLwheel_grf.y(), FLwheel_grf.z());
        // model_ptr->contact_force_[1] = Eigen::Vector3d(FRwheel_grf.x(), FRwheel_grf.y(), FRwheel_grf.z());
        // model_ptr->contact_force_[2] = Eigen::Vector3d(RLwheel_grf.x(), RLwheel_grf.y(), RLwheel_grf.z());
        // model_ptr->contact_force_[3] = Eigen::Vector3d(RRwheel_grf.x(), RRwheel_grf.y(), RRwheel_grf.z());

        //* Wheel Frame GRF
        model_ptr->contact_force_[0] = Eigen::Vector3d(FLwheel_grf.x()*cos(FLwheel_pos[2]) + FLwheel_grf.z()*sin(FLwheel_pos[2]), -FLwheel_grf.y(), -FLwheel_grf.x()*sin(FLwheel_pos[2]) + FLwheel_grf.z()*cos(FLwheel_pos[2]));
        model_ptr->contact_force_[1] = Eigen::Vector3d(FRwheel_grf.x()*cos(FRwheel_pos[2]) + FRwheel_grf.z()*sin(FRwheel_pos[2]), -FRwheel_grf.y(), -FRwheel_grf.x()*sin(FRwheel_pos[2]) + FRwheel_grf.z()*cos(FRwheel_pos[2]));
        model_ptr->contact_force_[2] = Eigen::Vector3d(RLwheel_grf.x()*cos(RLwheel_pos[2]) + RLwheel_grf.z()*sin(RLwheel_pos[2]), -RLwheel_grf.y(), -RLwheel_grf.x()*sin(RLwheel_pos[2]) + RLwheel_grf.z()*cos(RLwheel_pos[2]));
        model_ptr->contact_force_[3] = Eigen::Vector3d(RRwheel_grf.x()*cos(RRwheel_pos[2]) + RRwheel_grf.z()*sin(RRwheel_pos[2]), -RRwheel_grf.y(), -RRwheel_grf.x()*sin(RRwheel_pos[2]) + RRwheel_grf.z()*cos(RRwheel_pos[2]));


        model_ptr->contact_torque_[0] = Eigen::Vector3d(FLwheel_grt.x(), FLwheel_grt.y(), FLwheel_grt.z());
        model_ptr->contact_torque_[1] = Eigen::Vector3d(FRwheel_grt.x(), FRwheel_grt.y(), FRwheel_grt.z());
        model_ptr->contact_torque_[2] = Eigen::Vector3d(RLwheel_grt.x(), RLwheel_grt.y(), RLwheel_grt.z());
        model_ptr->contact_torque_[3] = Eigen::Vector3d(RRwheel_grt.x(), RRwheel_grt.y(), RRwheel_grt.z());

        model_ptr->sinkage_[0] = ter_FL.sinkage;
        model_ptr->sinkage_[1] = ter_FR.sinkage;
        model_ptr->sinkage_[2] = ter_RL.sinkage;
        model_ptr->sinkage_[3] = ter_RR.sinkage;


        // std::cout << "check" << model_ptr->contact_force_[0]  << std::endl;



        if (sys.GetChTime() >=1)
        {
          rover.BodyTrajectory(sys.GetChTime());
          motion_ctrl.Kin_Low_level_ctrl();

        //   motion_ctrl.longitudinal_vel_ctrl();
        //   motion_ctrl.Lateral_vel_ctrl(sys.GetChTime());

        //   motion_ctrl.drive_motor_control();
        //   motion_ctrl.steer_motor_control();



          for (int i = 0; i < 4; i++)
          {
            //! Drive Torque Mode, Steer, Sus fixed 되어 있는데 viper.cpp에서 설정해줄 수 있음
            driver->SetDriveMotorTorque(ctrl_ptr->drive_torque_[i],i);// FL(0), FR(1), RL(2), RR(3)
            driver->SetSteerMotorTorque(ctrl_ptr->steer_torque_[i],i);// FL(0), FR(1), RL(2), RR(3)
            // driver->SetSusMotorTorque(ctrl_ptr->sus_torque_[i],i);// FL(0), FR(1), RL(2), RR(3)
            // driver->SetDriveMotorTorque(5,i);
          }

        }

        else
        {
            for (int i = 0; i < 4; i++)
            {
                driver->SetDriveMotorTorque(0,i);
                driver->SetSteerMotorTorque(0,i);
                // driver->SetSusMotorTorque(0,i);
            }
        }



        std::cout << "Time: " << sys.GetChTime() << std::endl;


        vis->BeginScene();


        vis->Render();
        // tools::drawColorbar(vis.get(), 0, 20000, "Pressure yield [Pa]", 1180);
        vis->EndScene();
#endif
        //! Data Plotting
        data_logger.get_time(sys.GetChTime());
        data_logger.save_data();
        if (output) {
            // write drive torques of all four wheels into file
            csv << sys.GetChTime() << viper.GetWheelTracTorque(ViperWheelID::V_LF)
                << viper.GetWheelTracTorque(ViperWheelID::V_RF) << viper.GetWheelTracTorque(ViperWheelID::V_LB)
                << viper.GetWheelTracTorque(ViperWheelID::V_RB) << std::endl;
        }

        sys.DoStepDynamics(0.001); // 제어 Time Step
        viper.Update();
        // terrain.PrintStepStatistics(std::cout);
    }

    if (output) {
        csv.WriteToFile(out_dir + "/output.dat");
    }

    return 0;
}
