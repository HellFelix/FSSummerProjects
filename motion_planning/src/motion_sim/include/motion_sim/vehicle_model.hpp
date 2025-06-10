#ifndef VEHICLE_MODEL_H
#define VEHICLE_MODEL_H

#include <cmath>
#include <iostream>
#include <vector>

#include "Eigen/Dense"

// ================ Parent class ================
class VehicleModel {
  public:
    explicit VehicleModel(int dim_q, int dim_v,
                          rclcpp::Node::SharedPtr parent_node, double delta_max,
                          double delta_bias)
        : n_(dim_q), m_(dim_v), parent_node_(parent_node),
          delta_max_(delta_max), delta_bias_(delta_bias) {
        q_ = Eigen::VectorXd::Zero(n_);
        v_ = Eigen::VectorXd::Zero(m_);
        odom_.resize(2 * n_);
    }

    // Set specific value for q (also sets v to zero)
    void set_state(const std::vector<double> &q_new) {
        q_ = Eigen::Map<const Eigen::VectorXd>(q_new.data(), q_new.size());
        v_ = Eigen::VectorXd::Zero(m_);
    }

    // Call this function upon node configuration, it reads in the parameters
    virtual void initialize() = 0;

    // Getter for steering angle (visualization purposes)
    virtual double get_delta(const std::vector<double> &u) const = 0;

    // Odometry data (q, qdot) in that order
    Eigen::VectorXd get_odom() const { return odom_; }

    // Dynamic model update step
    void update(const std::vector<double> &u, double dt) {
        // k1
        Eigen::VectorXd k1_q = kinematics(q_, v_, u);
        Eigen::VectorXd k1_v = dynamics(q_, v_, u);
        odom_ << q_, k1_q; // Set odometry

        /* k2 ------------------------------------------------------------ */
        Eigen::VectorXd q2 = q_ + 0.5 * dt * k1_q;
        Eigen::VectorXd v2 = v_ + 0.5 * dt * k1_v;
        Eigen::VectorXd k2_q = kinematics(q2, v2, u); // q̇ at (q₂, v₂)
        Eigen::VectorXd k2_v = dynamics(q2, v2, u);   // v̇ at (q₂, v₂)

        /* k3 ------------------------------------------------------------ */
        Eigen::VectorXd q3 = q_ + 0.5 * dt * k2_q;
        Eigen::VectorXd v3 = v_ + 0.5 * dt * k2_v;
        Eigen::VectorXd k3_q = kinematics(q3, v3, u); // q̇ at (q₃, v₃)
        Eigen::VectorXd k3_v = dynamics(q3, v3, u);   // v̇ at (q₃, v₃)

        /* k4 ------------------------------------------------------------ */
        Eigen::VectorXd q4 = q_ + dt * k3_q;
        Eigen::VectorXd v4 = v_ + dt * k3_v;
        Eigen::VectorXd k4_q = kinematics(q4, v4, u); // q̇ at (q₄, v₄)
        Eigen::VectorXd k4_v = dynamics(q4, v4, u);   // v̇ at (q₄, v₄)

        /* State update -------------------------------------------------- */
        const double one_sixth = dt / 6.0;
        q_ += one_sixth * (k1_q + 2.0 * k2_q + 2.0 * k3_q + k4_q);
        v_ += one_sixth * (k1_v + 2.0 * k2_v + 2.0 * k3_v + k4_v);

        // std::cout << "===== q =====" << std::endl;
        // std::cout << q_ << std::endl;
        // std::cout << "===== v =====" << std::endl;
        // std::cout << v_ << std::endl;
    }

    virtual ~VehicleModel() = default;

  protected:
    // Dynamic model vectors and matrices (only default values here)
    virtual Eigen::MatrixXd G(const Eigen::VectorXd &q,
                              const std::vector<double> &u) {
        (void)q;
        (void)u;
        return Eigen::MatrixXd::Zero(n_, m_);
    }
    virtual Eigen::MatrixXd M(const Eigen::VectorXd &q) {
        (void)q;
        return Eigen::MatrixXd::Identity(m_, m_);
    }
    virtual Eigen::MatrixXd S(const Eigen::VectorXd &q,
                              const std::vector<double> &u) {
        (void)q;
        (void)u;
        return Eigen::MatrixXd::Zero(n_, m_);
    }
    virtual Eigen::VectorXd tau(const Eigen::VectorXd &q,
                                const Eigen::VectorXd &v,
                                const std::vector<double> &u) {
        (void)q;
        (void)v;
        (void)u;
        return Eigen::VectorXd::Zero(m_);
    }
    virtual Eigen::VectorXd m(const Eigen::VectorXd &q,
                              const Eigen::VectorXd &v) {
        (void)q;
        (void)v;
        return Eigen::VectorXd::Zero(m_);
    }
    // Kinematics overridable in case of kinematic model (u may enter into the
    // model non-trivially)
    virtual Eigen::VectorXd kinematics(const Eigen::VectorXd &q,
                                       const Eigen::VectorXd &v,
                                       const std::vector<double> &u) {
        Eigen::VectorXd qd = G(q, u) * v;
        return qd;
    }
    // Dynamic equation. Should not be overriden
    Eigen::VectorXd dynamics(const Eigen::VectorXd &q, const Eigen::VectorXd &v,
                             const std::vector<double> &u) {
        Eigen::VectorXd vd =
            M(q).inverse() *
            (G(q, u).transpose() * S(q, u) * tau(q, v, u) - m(q, v));
        return vd;
    }

    int n_; // Dimension of q
    int m_; // Dimension of v
    rclcpp::Node::SharedPtr
        parent_node_; // For declaring and getting parameters

    Eigen::VectorXd odom_; // Odometry vector
    Eigen::VectorXd q_;
    Eigen::VectorXd v_;

    // Always assumes linear steering model
    double delta_max_;
    double delta_bias_;
};

// ================ Derived classes ================

class KinematicBicycleModel : public VehicleModel {
  public:
    // q = [X Y theta]
    // u = [w delta]
    KinematicBicycleModel(rclcpp::Node::SharedPtr parent_node, double delta_max,
                          double delta_bias)
        : VehicleModel(3, 1, parent_node, delta_max, delta_bias) {
        // Declare parameters
        parent_node_->declare_parameter<double>("kbm.lf", 0.27);
        parent_node_->declare_parameter<double>("kbm.lr", 0.055);
    }

    void initialize() override {
        lf_ = parent_node_->get_parameter("kbm.lf").as_double();
        lr_ = parent_node_->get_parameter("kbm.lr").as_double();
    }

    double get_delta(const std::vector<double> &u) const override {
        return delta_max_ * u[1] + delta_bias_;
    }

  private:
    Eigen::MatrixXd G(const Eigen::VectorXd &q,
                      const std::vector<double> &u) override {
        double theta = q(2);
        double delta = delta_max_ * u[1] + delta_bias_;
        double beta = atan(lr_ / (lf_ + lr_) * tan(delta));

        Eigen::Vector3d G(cos(theta + beta), sin(theta + beta),
                          sin(beta) / lr_);
        return G;
    }

    Eigen::VectorXd kinematics(const Eigen::VectorXd &q,
                               const Eigen::VectorXd &v,
                               const std::vector<double> &u) {
        (void)v;
        double w = u[0];
        return G(q, u) * w;
    }

    // Params from config file
    double lf_; // Front wheelbase
    double lr_; // Rear wheelbase
};

class HybridBicycleModel : public VehicleModel {
  public:
    // q = [X Y theta]
    // v = [w]
    // u = [T delta]
    HybridBicycleModel(rclcpp::Node::SharedPtr parent_node, double delta_max,
                       double delta_bias)
        : VehicleModel(3, 1, parent_node, delta_max, delta_bias) {
        // Declare parameters
        parent_node_->declare_parameter<double>("hbm.mass", 4.75);
        parent_node_->declare_parameter<double>("hbm.Fmax", 5.0);
        parent_node_->declare_parameter<std::vector<double>>("hbm.drag_coeffs",
                                                             {0., 0., 0.});
        parent_node_->declare_parameter<double>("hbm.lf", 0.27);
        parent_node_->declare_parameter<double>("hbm.lr", 0.055);
    }

    void initialize() override {
        mass_ = parent_node_->get_parameter("hbm.mass").as_double();
        Fmax_ = parent_node_->get_parameter("hbm.Fmax").as_double();
        drag_coeffs_ =
            parent_node_->get_parameter("hbm.drag_coeffs").as_double_array();
        lf_ = parent_node_->get_parameter("hbm.lf").as_double();
        lr_ = parent_node_->get_parameter("hbm.lr").as_double();
    }

    double get_delta(const std::vector<double> &u) const override {
        return delta_max_ * u[1] + delta_bias_;
    }

  private:
    Eigen::MatrixXd G(const Eigen::VectorXd &q,
                      const std::vector<double> &u) override {
        double theta = q(2);
        double delta = delta_max_ * u[1] + delta_bias_;
        double beta = atan(lr_ / (lf_ + lr_) * tan(delta));

        Eigen::Vector3d G(cos(theta + beta), sin(theta + beta),
                          sin(beta) / lr_);
        return G;
    }
    Eigen::MatrixXd M(const Eigen::VectorXd &q) override {
        (void)q;
        Eigen::MatrixXd M(1, 1);
        M << mass_;
        return M;
    }
    Eigen::MatrixXd S(const Eigen::VectorXd &q,
                      const std::vector<double> &u) override {
        return G(q, u);
    }
    Eigen::VectorXd tau(const Eigen::VectorXd &q, const Eigen::VectorXd &v,
                        const std::vector<double> &u) override {
        (void)q;

        double T = Fmax_ * u[0];
        double w = v(0);

        double drag = drag_coeffs_[0] + drag_coeffs_[1] * std::fabs(w) +
                      drag_coeffs_[2] * w * w;
        drag = (w > 0) ? drag : -drag;

        Eigen::VectorXd tau(1);
        tau << T - drag;

        return tau;
    }

    // Params from config file
    double mass_;
    std::vector<double> drag_coeffs_;
    double lf_;   // Front wheelbase
    double lr_;   // Rear wheelbase
    double Fmax_; // Max thrust
};

class DynamicBicycleModel : public VehicleModel {
  public:
    // q = [X Y theta]
    // v = [vx vy omega]
    // u = [T delta]
    DynamicBicycleModel(rclcpp::Node::SharedPtr parent_node, double delta_max,
                        double delta_bias)
        : VehicleModel(3, 3, parent_node, delta_max, delta_bias) {
        // Declare parameters
        parent_node_->declare_parameter<double>("dbm.mass", 4.75);
        parent_node_->declare_parameter<double>("dbm.Iz", 1.0);
        parent_node_->declare_parameter<double>("dbm.Cf", 50000);
        parent_node_->declare_parameter<double>("dbm.Cr", 50000);
        parent_node_->declare_parameter<std::vector<double>>("dbm.drag_coeffs",
                                                             {0., 0., 0.});
        parent_node_->declare_parameter<double>("dbm.lf", 0.27);
        parent_node_->declare_parameter<double>("dbm.lr", 0.055);
    }

    void initialize() override {
        mass_ = parent_node_->get_parameter("dbm.mass").as_double();
        Iz_ = parent_node_->get_parameter("dbm.Iz").as_double();
        Cf_ = parent_node_->get_parameter("dbm.Cf").as_double();
        Cr_ = parent_node_->get_parameter("dbm.Cr").as_double();
        drag_coeffs_ =
            parent_node_->get_parameter("dbm.drag_coeffs").as_double_array();
        lf_ = parent_node_->get_parameter("dbm.lf").as_double();
        lr_ = parent_node_->get_parameter("dbm.lr").as_double();
    }

    double get_delta(const std::vector<double> &u) const override {
        return delta_max_ * u[1] + delta_bias_;
    }

  private:
    Eigen::MatrixXd G(const Eigen::VectorXd &q,
                      const std::vector<double> &u) override {
        (void)u;
        double theta = q(2);
        Eigen::MatrixXd G(3, 3);
        G << cos(theta), -sin(theta), 0, sin(theta), cos(theta), 0, 0, 0, 1;
        return G;
    }
    Eigen::MatrixXd M(const Eigen::VectorXd &q) override {
        (void)q;
        Eigen::MatrixXd M(3, 3);
        M << mass_, 0, 0, 0, mass_, 0, 0, 0, Iz_;
        return M;
    }
    Eigen::MatrixXd S(const Eigen::VectorXd &q,
                      const std::vector<double> &u) override {
        return G(q, u);
    }
    Eigen::VectorXd tau(const Eigen::VectorXd &q, const Eigen::VectorXd &v,
                        const std::vector<double> &u) override {
        (void)q;

        double Tf = 0.;
        double Tr = 20 * u[0];
        double delta = delta_max_ * u[1] + delta_bias_;

        double vx = v(0);
        double vy = v(1);
        double omega = v(2);

        double alpha_r = (std::fabs(vx) > 0.01)
                             ? atan((vy - lr_ * omega) / std::fabs(vx))
                             : 0.;
        double alpha_f = (std::fabs(vx) > 0.01)
                             ? atan((vy + lf_ * omega) / std::fabs(vx)) - delta
                             : 0.;
        double Fr = -10. * sin(1.4 * atan(alpha_r));
        double Ff = -10. * sin(1.4 * atan(alpha_f));
        double drag =
            drag_coeffs_[0] + drag_coeffs_[1] * vx + drag_coeffs_[2] * vx * vx;
        drag = (vx > 0) ? drag : -drag;

        Eigen::VectorXd tau(3);
        tau << Tr + cos(delta) * Tf - drag - Ff * sin(delta),
            Fr + Ff * cos(delta),
            -lr_ * Fr + lf_ * Ff * cos(delta) + lf_ * Tf * sin(delta);

        return tau;
    }
    Eigen::VectorXd m(const Eigen::VectorXd &q, const Eigen::VectorXd &v) {
        (void)q;

        double vx = v(0);
        double vy = v(1);
        double omega = v(2);

        Eigen::Vector3d m = {-mass_ * vy * omega, mass_ * vx * omega, 0.};
        return m;
    }

    double mass_;
    double Iz_;
    double lf_; // Front wheelbase
    double lr_; // Rear wheelbase
    double Cf_;
    double Cr_;
    std::vector<double> drag_coeffs_;
};

#endif // VEHICLE_MODEL_H
