/*
 * Copyright 2019 Giuseppe Silano, University of Sannio in Benevento, Italy
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 * 
 */

// Conversion functions between Eigen types and BebopS ROS message types.

#ifndef BEBOPS_MSGS_CONVERSIONS_H
#define BEBOPS_MSGS_CONVERSIONS_H

#include <ros/ros.h>

#include "bebopS/Sphinx.h"

namespace bebopS {

	inline void eigenSphinxFromMsg(const Sphinx& msg, EigenSphinx* sphinx_data_logging) {
	  assert(sphinx_data_logging != NULL);

	sphinx_data_logging->velXABC = msg.velXABC;                     
	sphinx_data_logging->velYABC = velYABC;                      
	sphinx_data_logging->velZABC = velZABC;                      
	sphinx_data_logging->angVelXABC = angVelXABC;                
	sphinx_data_logging->angVelYABC = angVelYABC;             
	sphinx_data_logging->angVelZABC = angVelZABC;              
	sphinx_data_logging->velXENU = velXENU;                     
	sphinx_data_logging->velYENU = velYENU;                     
	sphinx_data_logging->velZENU = velZENU;                     
	sphinx_data_logging->posX = posX;                        
	sphinx_data_logging->posY = posY;                        
	sphinx_data_logging->posZ = posZ;                        
	sphinx_data_logging->attitudeX = attitudeX;                  
	sphinx_data_logging->attitudeY = attitudeY;               
	sphinx_data_logging->attitudeZ = attitudeZ;                    
	sphinx_data_logging->accXABC = accXABC;                     
	sphinx_data_logging->accYABC = accYABC;                     
	sphinx_data_logging->accZABC = accZABC;                    
	sphinx_data_logging->angAccXABC = angAccXABC;                 
	sphinx_data_logging->angAccYABC = angAccYABC;                  
	sphinx_data_logging->angAccZABC = angAccZABC;                 
	
	}

}
