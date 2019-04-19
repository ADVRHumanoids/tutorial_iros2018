#include <cartesian_interface/CartesianInterfaceImpl.h>
#include <cartesian_interface/problem/Postural.h>
#include <IKProblem.h>


namespace XBot { namespace Cartesian {
   
    class SimpleIk : public CartesianInterfaceImpl {
        
    public:
        
        SimpleIk(ModelInterface::Ptr model);
        
        virtual bool update(double time, double period);
        
        virtual bool setBaseLink(const std::string& ee_name, 
                                 const std::string& new_base_link);
        
    private:
        
        static ProblemDescription get_task_list(const ModelInterface& model);
        
        OpenSoT::IKProblem::Ptr _opensot;
        
        Eigen::VectorXd _posture_ref, _dq, _q;
        
    };
    
    
} }


