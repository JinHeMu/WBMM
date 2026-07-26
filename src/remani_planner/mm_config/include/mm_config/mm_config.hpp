#ifndef _MM_CONFIG_HPP_
#define _MM_CONFIG_HPP_

#include <cmath>
#include <Eigen/Eigen>
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <memory>
#include <string>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "plan_env/grid_map.h"

namespace remani_planner
{
    class MMConfig
    {
    
    public:
        MMConfig() {}
        ~MMConfig() {}

        int getManiDof() const {
            return manipulator_dof_;
        }

        int getBaseDof() const {
            return mobile_base_dof_;
        }

        double getBaseWheelBase() const {
            return mobile_base_wheel_base_;
        }

        Eigen::Matrix4d getTq0(){
            return T_q_0_;
        }

        std::vector<Eigen::Matrix4Xd> getLinkPoint(){
            return manipulator_link_pts_;
        }

        std::vector<Eigen::VectorXd> getLinkRadii() const {
            return manipulator_link_radii_;
        }

        const std::vector<double>& getBaseCollisionRadii() const {
            return mobile_base_collision_radii_;
        }

        void getAJointTran(int joint_num, double theta, Eigen::Matrix4d &T, Eigen::Matrix4d &T_grad);

        void getJointTrans(const Eigen::VectorXd &theta, std::vector<Eigen::Matrix4d> &T_joint, std::vector<Eigen::Matrix4d> &T_joint_grad){
            T_joint.clear();
            T_joint_grad.clear();

            for(int i = 0; i < manipulator_dof_; ++i)
            {
               Eigen::Matrix4d T_temp, T_temp_grad;
               getAJointTran(i, theta(i), T_temp, T_temp_grad);
               T_joint.push_back(T_temp);
               T_joint_grad.push_back(T_temp_grad);
            }
        }

        Eigen::VectorXd getManiConfig(){
            return manipulator_config_;
        }

        void setParam(rclcpp::Node::SharedPtr node);
        void setParam(rclcpp::Node::SharedPtr node, const std::shared_ptr<GridMap>& env);
        bool checkCarObsCollision(Eigen::Vector3d car_state, bool precise, bool safe, double &min_dist);
        bool checkManiObsCollision(Eigen::Vector3d car_state, Eigen::VectorXd mani_state, bool safe, double &min_dist);
        bool checkCarManiCollision(Eigen::VectorXd mani_state, bool safe, double &min_dist);
        bool checkManiManiCollision(Eigen::VectorXd mani_state, bool safe, double &min_dist);
        bool checkManicollision(Eigen::Vector3d car_state, Eigen::VectorXd mani_state, bool safe);
        bool checkcollision(Eigen::Vector3d car_state, Eigen::VectorXd mani_state, bool safe);
        bool checkcollision(Eigen::Vector3d car_state, Eigen::VectorXd mani_state, bool safe, int &coll_type /*0: car, 1: mani, 2: car-mani, 3: mani-mani*/);
        void visMM(const rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr &pub, std::string ns, int idx, double alpha, const Eigen::Vector3d &car_state, const Eigen::VectorXd &joint_state, const bool &gripper_close);
        void visMMCheckBall(const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr &pub, std::string ns, int idx, double alpha, const Eigen::Vector3d &car_state, const Eigen::VectorXd &joint_state);
        void getMMMarkerArray(visualization_msgs::msg::MarkerArray &marker_array, std::string ns, int idx, double alpha, const Eigen::Vector3d &car_state, const Eigen::VectorXd &joint_state, const bool &gripper_close);
        double calYaw(const Eigen::Vector2d &vel_car, int traj_dir);
        Eigen::Matrix2d calR(const Eigen::Vector2d &vel_car, int traj_dir);
        Eigen::Matrix2d caldRldv(const Eigen::Vector2d &vel_car, const Eigen::Vector2d &delta_p, const int traj_dir);
        Eigen::Vector2d caldYawdV(const Eigen::Vector2d &vel_car);
        void getCarPts(const Eigen::Vector3d &car_state, std::vector<Eigen::Vector3d> &car_pts);
        void getCarPts(const Eigen::Vector3d &car_state, std::vector<Eigen::Vector3d> &car_pts, const Eigen::Vector3d &inflate_size);
        void getCarPtsGrad(const Eigen::Vector2d &pos_car, const Eigen::Vector2d &vel_car, const int traj_dir, const Eigen::Vector3d &inflate_size,
                            std::vector<Eigen::Vector3d> &car_pts, std::vector<Eigen::Matrix2d> &dPtdv);
        void getCarPtsGradNew(const Eigen::Vector2d &pos_car, const Eigen::Vector2d &vel_car, const int traj_dir, const Eigen::Vector3d &inflate_size,
                    std::vector<Eigen::Vector3d> &car_pts, std::vector<Eigen::Vector2d> &dPtdYaw);
        void CarState2T(const Eigen::Vector3d &car_state, Eigen::Matrix4d &T_car);
        void getJointTMat(const Eigen::VectorXd &theta, std::vector<Eigen::Matrix4d> &T_joint);
        void setGripperPoint(const bool gripper_close);
        double getBaseMaxVel() const {
            return mobile_base_max_vel_;
        }
        double getBaseMaxAcc() const {
            return mobile_base_max_acc_;
        }
        bool getUseFastArmer() const {
            return useFastArmer_;
        }
        
    private:
        struct UrdfJointKinematics
        {
            Eigen::Matrix4d origin = Eigen::Matrix4d::Identity();
            Eigen::Vector3d axis = Eigen::Vector3d::UnitZ();
        };

        rclcpp::Node::SharedPtr node_;
        std::vector<Eigen::Vector3d> color_set_;
        std::shared_ptr<GridMap> grid_map_;
        bool useFastArmer_;
        bool useUrdfModel_ = false;
        std::string urdfFile_;
        std::string urdfBaseLink_;
        std::string urdfMobileBaseCollisionLink_;
        std::vector<std::string> urdfJointNames_;
        std::vector<UrdfJointKinematics> urdfJointKinematics_;
        std::string visualizationFrame_ = "world";
        int mobile_base_dof_;
        int manipulator_dof_;
        double mobile_base_length_, mobile_base_width_, mobile_base_height_;
        double mobile_base_check_radius_;
        std::vector<Eigen::Vector3d> mobile_base_collision_centers_;
        std::vector<double> mobile_base_collision_radii_;
        double mobile_base_wheel_base_, mobile_base_wheel_radius_, mobile_base_max_wheel_omega_, mobile_base_max_wheel_alpha_;
        
        Eigen::VectorXd manipulator_config_;
        double manipulator_thickness_;
        double map_resolution_;
        std::vector<Eigen::Matrix4Xd> manipulator_link_pts_;
        std::vector<Eigen::VectorXd> manipulator_link_radii_;
        double car_safe_margin_;
        double mani_safe_margin_;
        double self_safe_margin_;
        double ground_safe_dis_;
        double mobile_base_max_vel_, mobile_base_max_acc_;

        Eigen::Matrix2d B_h_;
        Eigen::Matrix4d T_q_0_;
        int vis_idx_size_;

        Eigen::VectorXd manipulator_min_pos_, manipulator_max_pos_;

        visualization_msgs::msg::Marker sphere_free_, sphere_occ_;

        std::string mesh_resource_ur5_base_, mesh_resource_ur5_forearm_, mesh_resource_ur5_shoulder_, mesh_resource_ur5_upperarm_;
        std::string mesh_resource_ur5_wrist1_, mesh_resource_ur5_wrist2_, mesh_resource_ur5_wrist3_;
        std::string mesh_resource_mobile_base_, mesh_resource_fastarmer_base0_, mesh_resource_fastarmer_link1_, mesh_resource_fastarmer_link2_;
        std::string mesh_resource_fastarmer_link3_, mesh_resource_fastarmer_link4_, mesh_resource_fastarmer_link5_, mesh_resource_fastarmer_link6_;
        std::string mesh_resource_gripper_base_, mesh_resource_gripper_left_, mesh_resource_gripper_right_;

        void setColorSet();
        void setLinkPoint();
        bool loadUrdfModel();
        void visCarCheckBall(const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr &pub, std::string ns, int idx, double alpha, const Eigen::Vector3d &state);
        void visManiCheckBall(const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr &pub, std::string ns, int idx, double alpha, const Eigen::Vector3d &car_state, const Eigen::VectorXd &joint_state);
        void visMesh(const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr &pub, int id, std::string ns, double alpha, Eigen::Vector3d color_rgb, const Eigen::Matrix4d &T, const std::string &mesh_file);
        
        visualization_msgs::msg::MarkerArray getCarMarkerArray(std::string ns, int idx, double alpha, const Eigen::Vector3d &state);
        visualization_msgs::msg::MarkerArray getManiMarkerArray(std::string ns, int idx, double alpha, const Eigen::Vector3d &car_state, const Eigen::VectorXd &joint_state, const bool &gripper_close);
        visualization_msgs::msg::Marker getMarker(int id, std::string ns, double alpha, const Eigen::Matrix4d &T, const std::string &mesh_file);

        Eigen::Matrix3d euler2rotation(double r, double p, double y){
            Eigen::Quaterniond quad = Eigen::AngleAxisd(r, Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(p, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(y, Eigen::Vector3d::UnitZ());
            return quad.toRotationMatrix();
        }

    public:
        // typedef std::unique_ptr<MMConfig> Ptr;
        typedef std::shared_ptr<MMConfig> Ptr;
    };

    struct MMState{
        double time_stamp;
        Eigen::Vector2d car_pos;
        Eigen::Vector2d car_vel;
        double car_yaw;
        double car_v; // >0 foreward <0 backward
        double car_a; // >0 foreward <0 backward
        double car_omega;
        double car_domega;
        Eigen::Matrix2d car_R;
        double car_input_vel;
        double car_input_omega;
        Eigen::VectorXd mani_pos;
        Eigen::VectorXd mani_vel;

        Eigen::Vector3d getCarXYYaw(){
            return Eigen::Vector3d(car_pos(0), car_pos(1), car_yaw);
        }

        Eigen::Vector4d getCarXYYawV(){
            return Eigen::Vector4d(car_pos(0), car_pos(1), car_yaw, car_v);
        }
    };

} // namespace remani_planner

#endif
