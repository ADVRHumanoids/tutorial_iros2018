#include <urdf_parser/urdf_parser.h>
#include <iostream>
#include <fstream>


int main(int argc, char ** argv)
{
    if(argc != 2)
    {
        std::cout << "USAGE:\n" << argv[0] << " path_to_urdf" << std::endl;
        exit(1);
    }
    
    std::string urdf_file(argv[1]);
    
    auto urdfdom = urdf::parseURDFFile(urdf_file);
    
    if(!urdfdom)
    {
        std::cout << "Unable to parse URDF " << urdf_file << std::endl;
        exit(1);
    }
    
    std::ofstream myfile;
    myfile.open ("gain_map.yaml");
    
    int i = 1;
    for(auto pair : urdfdom->joints_)
    {
        std::string jname = pair.first;
        urdf::JointSharedPtr joint = pair.second;
        
        if(joint->type == urdf::Joint::REVOLUTE || joint->type == urdf::Joint::PRISMATIC)
        {
            myfile << jname << ": " << i++ << "\n";
        }
    }
    
    myfile.close();
    
    myfile.open ("joint_map.yaml");
    
    i = 1;
    for(auto pair : urdfdom->joints_)
    {
        std::string jname = pair.first;
        urdf::JointSharedPtr joint = pair.second;
        
        if(joint->type == urdf::Joint::REVOLUTE || joint->type == urdf::Joint::PRISMATIC)
        {
            myfile << i++ << ": " << jname << "\n";
        }
    }
    
    myfile.close();
    
    std::cout << "Generated gain_map.yaml & joint_map.yaml" << std::endl;
}