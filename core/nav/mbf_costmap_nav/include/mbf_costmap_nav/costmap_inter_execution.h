/*
 *  Copyright 2018, Magazino GmbH, Sebastian Pütz, Jorge Santos Simón
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *  1. Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *
 *  3. Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *  costmap_inter_execution.h
 *
 *  authors:
 *    Sebastian Pütz <spuetz@uni-osnabrueck.de>
 *    Jorge Santos Simón <santos@magazino.eu>
 *
 */

#ifndef MBF_COSTMAP_NAV__COSTMAP_INTER_EXECUTION_H_
#define MBF_COSTMAP_NAV__COSTMAP_INTER_EXECUTION_H_

#include <mbf_abstract_nav/abstract_inter_execution.h>
#include <mbf_costmap_core/costmap_inter.h>

#include "mbf_costmap_nav/MoveBaseFlexConfig.h"
#include "mbf_costmap_nav/costmap_wrapper.h"


namespace mbf_costmap_nav
{
/**
 * @brief The CostmapInterExecution binds a global costmap to the AbstractInterExecution
 *
 * @ingroup inter_execution move_base_server
 */
class CostmapInterExecution : public mbf_abstract_nav::AbstractInterExecution
{
public:
  /**
   * @brief Constructor.
   * @param inter_name Name of the inter to use.
   * @param inter_ptr Shared pointer to the plugin to use.
   * @param robot_info Current robot state
   * @param global_costmap_ptr Shared pointer to the global costmap.
   * @param local_costmap_ptr Shared pointer to the local costmap.
   * @param config Current server configuration (dynamic).
   */
  CostmapInterExecution(const std::string& inter_name,
                          const mbf_costmap_core::CostmapInter::Ptr& inter_ptr,
                          const mbf_utility::RobotInformation& robot_info,
                          const ros::Publisher &goal_pub,
                          const CostmapWrapper::Ptr& global_costmap_ptr,
                          const CostmapWrapper::Ptr& local_costmap_ptr,
                          const MoveBaseFlexConfig& config);

  /**
   * @brief Destructor
   */
  virtual ~CostmapInterExecution();


private:
  /**
   * @brief Implementation-specific setup function, called right before execution.
   * This method overrides abstract execution empty implementation with underlying map-specific setup code.
   */
  void preRun()
  {
    global_costmap_ptr_->checkActivate();
    local_costmap_ptr_->checkActivate();
  };

  /**
   * @brief Implementation-specific cleanup function, called right after execution.
   * This method overrides abstract execution empty implementation with underlying map-specific cleanup code.
   */
  void postRun()
  {
    global_costmap_ptr_->checkDeactivate();
    local_costmap_ptr_->checkDeactivate();
  };

  mbf_abstract_nav::MoveBaseFlexConfig toAbstract(const MoveBaseFlexConfig &config);

  /**
   * @brief Calls the inter plugin to make a plan from the start pose to the goal pose.
   * @param start The start pose for planning
   * @param goal The goal pose for planning
   * @param plan The computed plan by the plugin
   * @param cost The computed costs for the corresponding plan
   * @param message An optional message which should correspond with the returned outcome
   * @return An outcome number, see also the action definition in the GetInterPath.action file
   */
  virtual uint32_t makePlan(
      const geometry_msgs::PoseStamped &start,
      const geometry_msgs::PoseStamped &goal,
      std::vector<geometry_msgs::PoseStamped> &plan,
      double &cost,
      std::string &message);

  //! Shared pointer to both global and local costmaps
  const CostmapWrapper::Ptr &global_costmap_ptr_;
  const CostmapWrapper::Ptr &local_costmap_ptr_;

  //! Whether to lock costmaps before calling the inter (see issue #4 for details)
  bool lock_costmap_;

  //! Name of the inter assigned by the class loader
  std::string inter_name_;
};

} /* namespace mbf_costmap_nav */

#endif /* MBF_COSTMAP_NAV__COSTMAP_INTER_EXECUTION_H_ */
