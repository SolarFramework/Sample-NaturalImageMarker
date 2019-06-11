/**
 * @copyright Copyright (c) 2017 B-com http://www.b-com.com/
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <boost/log/core.hpp>
#include "core/Log.h"
#include "xpcf/xpcf.h"


// ADD COMPONENTS HEADERS HERE, e.g #include "SolarComponent.h"

#include "SolARPluginPipelineManager.h"

using namespace SolAR;
using namespace SolAR::PIPELINE;

namespace xpcf  = org::bcom::xpcf;

#include "SolARModuleOpencv_traits.h"
#include "SolARImageViewerOpencv.h"
#include "SolAR3DOverlayBoxOpencv.h"

using namespace SolAR;
using namespace SolAR::MODULES::OPENCV;
using namespace SolAR::api;

int main(){

#if NDEBUG
    boost::log::core::get()->set_logging_enabled(false);
#endif

    LOG_ADD_LOG_TO_CONSOLE();

    SolARPluginPipelineManager pipeline;
    if (pipeline.init("PipelineNaturalImageMarker.xml", "6f6c7ae8-764c-49b3-a2fc-e9a4f539b9b1"))
    {
        auto imageViewerResult = xpcf::getComponentManagerInstance()->create<MODULES::OPENCV::SolARImageViewerOpencv>()->bindTo<display::IImageViewer>();
        auto overlay3DComponent = xpcf::getComponentManagerInstance()->create<MODULES::OPENCV::SolAR3DOverlayBoxOpencv>()->bindTo<display::I3DOverlay>();

        // Set camera parameters
        CamDistortion  distorsion_param = CamDistortion::Zero();
        CamCalibration calib = pipeline.getCameraParameters();

        overlay3DComponent->setCameraParameters(calib, distorsion_param);

        unsigned char* r_imageData=new unsigned char[640*480*3];
        SRef<Image> camImage=xpcf::utils::make_shared<Image>(r_imageData,640,480,SolAR::Image::LAYOUT_BGR,SolAR::Image::INTERLEAVED,SolAR::Image::TYPE_8U);

        Transform3Df s_pose;

        if (pipeline.start(camImage->data()))
        {
            while (true)
            {
                Transform3Df pose;

                PIPELINEMANAGER_RETURNCODE returnCode = pipeline.udpate(pose);
                if(returnCode==PIPELINEMANAGER_RETURNCODE::_ERROR)
                    break;

                if ((returnCode & PIPELINEMANAGER_RETURNCODE::_NEW_POSE))
                {
                    for(int i=0;i<3;i++)
                         for(int j=0;j<3;j++)
                             s_pose(i,j)=pose.rotation().coeff(i,j);
                    for(int i=0;i<3;i++)
                             s_pose(i,3)=pose.translation().coeff(i,0);
                    for(int j=0;j<3;j++)
                        s_pose(3,j)=0;
                    s_pose(3,3)=1;
                    overlay3DComponent->draw(s_pose, camImage);
                }

                if (imageViewerResult->display(camImage) == SolAR::FrameworkReturnCode::_STOP){
                    pipeline.stop();
                    break;
                }
             }
        }
        delete[] r_imageData;
    }
}





