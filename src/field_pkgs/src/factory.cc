#include <ignition/math/Pose3.hh>
#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"

namespace gazebo
{
    class Factory : public WorldPlugin
    {
    public:
        void Load(physics::WorldPtr _parent, sdf::ElementPtr /*_sdf*/)
        {
            // Option 1: Insert model from file via function call.
            // The filename must be in the GAZEBO_MODEL_PATH environment variable.
            _parent->InsertModelFile("model://box");
            //_parent->InsertModelFile("model://Desk");

            // Option 2: Insert model from string via function call.
            // Insert a sphere model from string
            sdf::SDF sphereSDF;
            sphereSDF.SetFromString(
                "<sdf version ='1.4'>\
                  <model name ='sphere'>\
                    <pose>2 3 0 0 0 0</pose>\
                    <link name ='link'>\
                      <pose>0 0 .5 0 0 0</pose>\
                      <visual name ='visual'>\
                        <geometry>\
                          <sphere><radius>0.5</radius></sphere>\
                        </geometry>\
                      </visual>\
                    </link>\
                  </model>\
                </sdf>");
            // Demonstrate using a custom model name.
            sdf::ElementPtr model = sphereSDF.Root()->GetElement("model");
            model->GetAttribute("name")->SetFromString("unique_sphere");
            _parent->InsertModelSDF(sphereSDF);

            std::cout<<"Load field_pkgs Factory Plugin..."<<std::endl;
        }
    };
    // Register this plugin with the simulator
    GZ_REGISTER_WORLD_PLUGIN(Factory)
}