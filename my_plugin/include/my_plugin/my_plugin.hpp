#ifndef GAZEBO_PLUGINS__MY_PLUGIN_HPP
#define GAZEBO_PLUGINS__MY_PLUGIN_HPP

#include "gazebo/common/Plugin.hh"
#include <memory>

using namespace std;


namespace gazebo_plugins
{

class MyPluginPrivate;

class MyPlugin: public gazebo::ModelPlugin{

    public:
    MyPlugin();
    ~MyPlugin();

    protected:
    virtual void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf);
    virtual void Update(const gazebo::common::UpdateInfo & _info);
    virtual void Reset();

    private:
    unique_ptr<MyPluginPrivate> impl_;

};


} // namespace gazebo_plugins

#endif // GAZEBO_PLUGINS__MY_PLUGIN_HPP