#ifndef GZ_PLUGINS__MECANUM_DRIVE_CONTROLLER_HPP
#define GZ_PLUGINS__MECANUM_DRIVE_CONTROLLER_HPP

#include "gazebo/common/Plugin.hh"
#include <memory>


namespace gz_plugins
{
class MecanumDriveControllerPrivate;

class MecanumDriveController: public gazebo::ModelPlugin{
    public:
    MecanumDriveController();
    ~MecanumDriveController();

    protected:
    void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf) override; // ignore the squiggly here vs code issues
    void Reset() override;


    private:
    std::unique_ptr<MecanumDriveControllerPrivate> impl_;


};



}
#endif