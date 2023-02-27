#include <mlmap.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

namespace mlmapping_ns
{

    using namespace std;

    class MlMapNodeletClass : public nodelet::Nodelet
    {
    public:
        MlMapNodeletClass() { ; }
        ~MlMapNodeletClass() { ; }

    private:
        mlmap::Ptr mapPtr;
        virtual void onInit()
        {
            cout << "ML mapping node launched!" << endl;
            ros::NodeHandle nh = getMTPrivateNodeHandle();
            mapPtr.reset(new mlmap);
            mapPtr->init_map(nh);
            // ros::Timer timer1 = nh.createTimer(ros::Duration(0.1), boost::bind(&mlmap::timerCb1, mapPtr));
            
        }
    }; // class AwarenessMapNodeletClass
} // namespace mlmapping_ns

PLUGINLIB_EXPORT_CLASS(mlmapping_ns::MlMapNodeletClass, nodelet::Nodelet)
