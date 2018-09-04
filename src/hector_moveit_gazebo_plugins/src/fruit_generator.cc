#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/Mesh.hh>
#include <gazebo/common/ColladaLoader.hh>
#include <ignition/math/Vector3.hh>

#include <iostream>

#define TOTAL_FRUIT_NUMBER 120
namespace gazebo
{
  class FruitGenerator : public WorldPlugin
  {
    public: void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf)
    {
        common::ColladaLoader loader;
        const char *homedir;
        homedir = getenv("HOME");
        std::string home = homedir;
        home.append("/.gazebo/models/oak_tree/meshes/oak_tree.dae"); // Assume all trees are oak trees.
        common::Mesh* mesh = loader.Load(home);
        std::vector<ignition::math::Vector3d> tree_origins;
        auto models = _parent->Models();

        for(auto m : models){
            std::string model_name = m->GetName();
            if(model_name.find("tree")!=std::string::npos)//It is a tree then
                tree_origins.push_back(m->WorldPose().Pos());
        }

        float* vertices;
        int* indices;
        vertices = (float *)malloc(sizeof(float) * mesh->GetVertexCount() * 3);
        indices = (int *)malloc(sizeof(int) * mesh->GetIndexCount() * 3);
        mesh->FillArrays(&vertices,&indices);
        std::string model_name = "apple";
        std::string apple_path = homedir;
        apple_path.append("/.gazebo/models/apple/model.sdf");
        std::ifstream f(apple_path);
        std::stringstream buf;
        buf << f.rdbuf();
        std::string model_xml = buf.str();
        sdf::SDF appleSDF;
        appleSDF.SetFromString(model_xml);
        
        std::vector<std::vector<float> > positions;
        for(int i=0;i<mesh->GetVertexCount()*3;i+=3){
            double planar_distance = sqrt(pow(vertices[i],2) + pow(vertices[i+1],2));
            if(vertices[i+2]>2.0 && vertices[i+2]<4.5 && planar_distance < 2.0){//Only consider the vertices higher than 2 meters and lower than 4.5
                // Also exclude the vertices too far than the tree root.
                std::vector<float> position = {vertices[i],vertices[i+1],vertices[i+2]};
                positions.push_back(position);
            }
        }
        std::vector<int> random_indices(TOTAL_FRUIT_NUMBER,-1);//There exists 50 apples
        for(int i=0;i<TOTAL_FRUIT_NUMBER;i++){
            int r;
            do{
                r = rand() % TOTAL_FRUIT_NUMBER;
            }while(std::find(random_indices.begin(),random_indices.end(),r)!=random_indices.end());
            random_indices[i] = r;
        }
        int cnt=0;
        for(int i=0;i<TOTAL_FRUIT_NUMBER;i++){
            // Randomly choose which tree to assign the apple
            int tree;
            tree = rand() % tree_origins.size();
            //std::cout<<"\033[1;32m Assigning to tree "<<tree<<"\033[0m\n";
            int origx = tree_origins[tree].X();
            int origy = tree_origins[tree].Y();
            int origz = tree_origins[tree].Z();
            sdf::ElementPtr apple = appleSDF.Root()->GetElement("model");
            std::string tmp = model_name;
            tmp.append(std::to_string(cnt++));
            apple->GetAttribute("name")->SetFromString(tmp);
            apple->GetElement("pose")->Set(ignition::math::Pose3d(
                            ignition::math::Vector3d(origx + positions[random_indices[i]][0],origy + positions[random_indices[i]][1],origz + positions[random_indices[i]][2]),ignition::math::Quaternion<double>(0,0,0,1) ));
            _parent->InsertModelSDF(appleSDF);
        }
        free(vertices);
        free(indices);
    }

    
  };

  // Register this plugin with the simulator
  GZ_REGISTER_WORLD_PLUGIN(FruitGenerator)
}