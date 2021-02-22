#ifndef PIPS_EGOCYLINDRICAL_FREE_SPACE_CHECKER_H
#define PIPS_EGOCYLINDRICAL_FREE_SPACE_CHECKER_H

#include <geometry_msgs/Pose.h>

namespace pips_egocylindrical
{

class FreeSpaceChecker
{
public:
  
  virtual float getFreeSpaceRadius(geometry_msgs::Pose pose)=0;
  
};
  

} //end namespace pips_egocylindrical
  


#endif //PIPS_EGOCYLINDRICAL_FREE_SPACE_CHECKER_H
