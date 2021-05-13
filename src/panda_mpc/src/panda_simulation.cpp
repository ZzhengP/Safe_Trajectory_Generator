//
// Created by zheng on 12/05/2021.
//

#include <gazebo-9/gazebo/gazebo.hh>
#include <gazebo-9/gazebo/physics/physics.hh>
#include <gazebo-9/gazebo/common/common.hh>
#include <iostream>


namespace gazebo{

    class PandaSimulation: public ModelPlugin
    {

    public:
          PandaSimulation():ModelPlugin()
          {

              std::cout << "hello world " << std::endl;
          }


         void Load(physics::ModelPtr model, sdf::ElementPtr /*sdf*/){
             std::cout << "hello world " << std::endl;

         }


    };
    GZ_REGISTER_MODEL_PLUGIN(PandaSimulation)
}