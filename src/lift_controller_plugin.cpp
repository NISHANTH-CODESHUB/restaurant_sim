#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

namespace gazebo
{
  // Timeline (seconds), one full cycle:
  //   0.0 - 2.0   doors opening at ground   (ground + cabin doors)
  //   2.0 - 6.0   dwell, doors open at ground
  //   6.0 - 8.0   doors closing at ground
  //   8.0 - 14.0  cabin rising ground -> first floor
  //  14.0 - 16.0  doors opening at first floor (first + cabin doors)
  //  16.0 - 20.0  dwell, doors open at first floor
  //  20.0 - 22.0  doors closing at first floor
  //  22.0 - 28.0  cabin descending first floor -> ground
  // (loops back to 0.0)
  static const double T_DOOR = 2.0;
  static const double T_DWELL = 4.0;
  static const double T_TRAVEL = 6.0;
  static const double CYCLE =
      T_DOOR + T_DWELL + T_DOOR + T_TRAVEL + T_DOOR + T_DWELL + T_DOOR + T_TRAVEL; // 28.0

  static const double DOOR_MAX = 0.85;   // meters, fully open
  static const double LIFT_MAX = 3.5;    // meters, ground -> first floor
  static const double LIFT_SPEED = LIFT_MAX / T_TRAVEL; // m/s while travelling
  static const double LIFT_FMAX = 5000;  // matches lift_vertical_joint effort limit

  class LiftControllerPlugin : public ModelPlugin
  {
    public:
      void Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/) override
      {
        this->model = _model;

        this->jGroundDoorLeft  = this->model->GetJoint("ground_door_left_joint");
        this->jGroundDoorRight = this->model->GetJoint("ground_door_right_joint");
        this->jFirstDoorLeft   = this->model->GetJoint("first_door_left_joint");
        this->jFirstDoorRight  = this->model->GetJoint("first_door_right_joint");
        this->jCabinDoorLeft   = this->model->GetJoint("cabin_door_left_joint");
        this->jCabinDoorRight  = this->model->GetJoint("cabin_door_right_joint");
        this->jLift            = this->model->GetJoint("lift_vertical_joint");

        if (!this->jGroundDoorLeft || !this->jGroundDoorRight ||
            !this->jFirstDoorLeft  || !this->jFirstDoorRight  ||
            !this->jCabinDoorLeft  || !this->jCabinDoorRight  ||
            !this->jLift)
        {
          gzerr << "[lift_controller] one or more expected joints not found on model '"
                << this->model->GetName() << "' - plugin will not run.\n";
          return;
        }

        this->startTime = this->model->GetWorld()->SimTime();

        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
            std::bind(&LiftControllerPlugin::OnUpdate, this));
      }

    private:
      // Linear interpolation helper, clamped to [0, target]
      double Ramp(double elapsedInPhase, double phaseDuration, double target)
      {
        double frac = elapsedInPhase / phaseDuration;
        if (frac < 0.0) frac = 0.0;
        if (frac > 1.0) frac = 1.0;
        return frac * target;
      }

      void SetDoors(physics::JointPtr left, physics::JointPtr right, double value)
      {
        left->SetPosition(0, value);
        right->SetPosition(0, value);
      }

      void OnUpdate()
      {
        if (!this->jLift) return; // Load() bailed out earlier

        common::Time now = this->model->GetWorld()->SimTime();
        double elapsed = (now - this->startTime).Double();
        double t = std::fmod(elapsed, CYCLE);
        if (t < 0) t += CYCLE;

        double doorGround = 0.0;
        double doorFirst = 0.0;
        double doorCabin = 0.0;
        double liftVel = 0.0; // target velocity for the cabin's motor (0 = hold in place)

        double boundary1 = T_DOOR;                               // 2.0  end of opening at ground
        double boundary2 = boundary1 + T_DWELL;                  // 6.0  end of dwell at ground
        double boundary3 = boundary2 + T_DOOR;                   // 8.0  end of closing at ground
        double boundary4 = boundary3 + T_TRAVEL;                 // 14.0 end of rising
        double boundary5 = boundary4 + T_DOOR;                   // 16.0 end of opening at first
        double boundary6 = boundary5 + T_DWELL;                  // 20.0 end of dwell at first
        double boundary7 = boundary6 + T_DOOR;                   // 22.0 end of closing at first
        // boundary8 = boundary7 + T_TRAVEL == CYCLE               end of descending

        if (t < boundary1)
        {
          // Opening at ground
          double v = this->Ramp(t, T_DOOR, DOOR_MAX);
          doorGround = v; doorCabin = v; doorFirst = 0.0;
          liftVel = 0.0; // hold at ground
        }
        else if (t < boundary2)
        {
          // Dwell, open at ground
          doorGround = DOOR_MAX; doorCabin = DOOR_MAX; doorFirst = 0.0;
          liftVel = 0.0; // hold at ground
        }
        else if (t < boundary3)
        {
          // Closing at ground
          double v = DOOR_MAX - this->Ramp(t - boundary2, T_DOOR, DOOR_MAX);
          doorGround = v; doorCabin = v; doorFirst = 0.0;
          liftVel = 0.0; // hold at ground
        }
        else if (t < boundary4)
        {
          // Rising
          doorGround = 0.0; doorCabin = 0.0; doorFirst = 0.0;
          liftVel = LIFT_SPEED;
        }
        else if (t < boundary5)
        {
          // Opening at first floor
          double v = this->Ramp(t - boundary4, T_DOOR, DOOR_MAX);
          doorFirst = v; doorCabin = v; doorGround = 0.0;
          liftVel = 0.0; // hold at first floor
        }
        else if (t < boundary6)
        {
          // Dwell, open at first floor
          doorFirst = DOOR_MAX; doorCabin = DOOR_MAX; doorGround = 0.0;
          liftVel = 0.0; // hold at first floor
        }
        else if (t < boundary7)
        {
          // Closing at first floor
          double v = DOOR_MAX - this->Ramp(t - boundary6, T_DOOR, DOOR_MAX);
          doorFirst = v; doorCabin = v; doorGround = 0.0;
          liftVel = 0.0; // hold at first floor
        }
        else
        {
          // Descending
          doorGround = 0.0; doorCabin = 0.0; doorFirst = 0.0;
          liftVel = -LIFT_SPEED;
        }

        // Drive the cabin with a real velocity motor rather than teleporting its
        // position: the cabin's joint axis is vertical (parallel to gravity), and
        // with no motor a SetPosition() teleport is immediately undone by gravity
        // acting on the 300 kg cabin before the next render. The horizontal doors
        // don't have this problem since gravity has no component along their
        // sliding axis, which is why they worked while the cabin did not.
        // fmax must be set every step because Gazebo clears the motor force cap
        // each time a joint update occurs; without it, "vel" is silently ignored.
        this->jLift->SetParam("vel", 0, liftVel);
        this->jLift->SetParam("fmax", 0, LIFT_FMAX);

        this->SetDoors(this->jGroundDoorLeft, this->jGroundDoorRight, doorGround);
        this->SetDoors(this->jFirstDoorLeft, this->jFirstDoorRight, doorFirst);
        this->SetDoors(this->jCabinDoorLeft, this->jCabinDoorRight, doorCabin);
      }

      physics::ModelPtr model;
      physics::JointPtr jGroundDoorLeft, jGroundDoorRight;
      physics::JointPtr jFirstDoorLeft, jFirstDoorRight;
      physics::JointPtr jCabinDoorLeft, jCabinDoorRight;
      physics::JointPtr jLift;
      common::Time startTime;
      event::ConnectionPtr updateConnection;
  };

  GZ_REGISTER_MODEL_PLUGIN(LiftControllerPlugin)
}
