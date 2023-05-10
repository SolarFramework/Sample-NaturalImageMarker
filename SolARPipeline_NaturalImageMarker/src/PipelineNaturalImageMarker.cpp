/**
 * @copyright Copyright (c) 2015 All Right Reserved, B-com http://www.b-com.com/
 *
 * This file is subject to the B<>Com License.
 * All other rights reserved.
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 */

#include "xpcf/module/ModuleFactory.h"
#include "PipelineNaturalImageMarker.h"

#include "core/Log.h"

#define TRACKING

namespace xpcf=org::bcom::xpcf;

// Declaration of the module embedding the fiducial marker pipeline
XPCF_DECLARE_MODULE("5bbc1452-8d7f-4512-83f1-7bf48453809f", "SolARPipelineNaturalImageMarker", "The module embedding a pipeline to estimate the pose based on a natural image marker")

extern "C" XPCF_MODULEHOOKS_API xpcf::XPCFErrorCode XPCF_getComponent(const boost::uuids::uuid& componentUUID,SRef<xpcf::IComponentIntrospect>& interfaceRef)
{
    xpcf::XPCFErrorCode errCode = xpcf::XPCFErrorCode::_FAIL;
    errCode = xpcf::tryCreateComponent<SolAR::PIPELINES::PipelineNaturalImageMarker>(componentUUID,interfaceRef);

    return errCode;
}

XPCF_BEGIN_COMPONENTS_DECLARATION
XPCF_ADD_COMPONENT(SolAR::PIPELINES::PipelineNaturalImageMarker)
XPCF_END_COMPONENTS_DECLARATION


