////////////////////////////////////////////////////////////////////////////////////////////////////
//                               This file is part of CosmoScout VR                               //
//      and may be used under the terms of the MIT license. See the LICENSE file for details.     //
//                        Copyright: (c) 2019 German Aerospace Center (DLR)                       //
////////////////////////////////////////////////////////////////////////////////////////////////////

#include "Satellite.hpp"

#include <VistaKernel/GraphicsManager/VistaNodeBridge.h>
#include <VistaKernel/VistaSystem.h>
#include <VistaKernelOpenSGExt/VistaOpenSGMaterialTools.h>

#include "../../../src/cs-core/Settings.hpp"
#include "../../../src/cs-core/SolarSystem.hpp"
#include "../../../src/cs-graphics/GltfLoader.hpp"
#include "../../../src/cs-utils/convert.hpp"
#include "../../../src/cs-utils/utils.hpp"

#include <glm/gtc/type_ptr.hpp>

namespace csp::satellites {

////////////////////////////////////////////////////////////////////////////////////////////////////

Satellite::Satellite(Plugin::Settings::Satellite const& config, std::string const& sCenterName,
    std::string const& sFrameName, double tStartExistence, double tEndExistence,
    VistaSceneGraph* sceneGraph, std::shared_ptr<cs::core::Settings> const& settings,
    std::shared_ptr<cs::core::SolarSystem> const& solarSystem)
    : cs::scene::CelestialBody(sCenterName, sFrameName, tStartExistence, tEndExistence)
    , mSceneGraph(sceneGraph)
    , mSettings(settings)
    , mSolarSystem(solarSystem)
    , mModel(std::make_unique<cs::graphics::GltfLoader>(
          config.mModelFile, config.mEnvironmentMap, true))
    , mSize(config.mSize) {

  // TODO: make configurable
  pVisibleRadius = 10000;

  mModel->setLightIntensity(15.0);
  mModel->setIBLIntensity(1.5);
  mModel->setLightColor(1.0, 1.0, 1.0);

  mAnchor.reset(sceneGraph->NewTransformNode(sceneGraph->GetRoot()));
  mTransform.reset(sceneGraph->NewTransformNode(mAnchor.get()));

  if (config.mTransformation) {
    auto scale = (float)config.mTransformation->mScale;
    mTransform->SetScale(scale, scale, scale);
    setAnchorPosition(config.mTransformation->mTranslation);
    setAnchorRotation(config.mTransformation->mRotation);
  }

  mModel->attachTo(sceneGraph, mTransform.get());

  VistaOpenSGMaterialTools::SetSortKeyOnSubtree(
      mTransform.get(), static_cast<int>(cs::utils::DrawOrder::eOpaqueItems));

  mTransform->SetIsEnabled(false);
}

////////////////////////////////////////////////////////////////////////////////////////////////////

Satellite::~Satellite() {
  mSceneGraph->GetRoot()->DisconnectChild(mTransform.get());
  mSceneGraph->GetRoot()->DisconnectChild(mAnchor.get());
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void Satellite::setSun(std::shared_ptr<const cs::scene::CelestialObject> const& sun) {
  mSun = sun;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

bool Satellite::getIntersection(
    glm::dvec3 const& rayPos, glm::dvec3 const& rayDir, glm::dvec3& pos) const {
  return false;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

double Satellite::getHeight(glm::dvec2 lngLat) const {
  return 0;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

glm::dvec3 Satellite::getRadii() const {
  return glm::dvec3(mSize);
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void Satellite::update(double tTime, cs::scene::CelestialObserver const& oObs) {
  cs::scene::CelestialBody::update(tTime, oObs);

  mTransform->SetIsEnabled(getIsInExistence() && pVisible.get());

  if (getIsInExistence() && pVisible.get()) {
    mAnchor->SetTransform(glm::value_ptr(matWorldTransform), true);

    if (mSun) {
      float sunIlluminance(1.f);
      auto  ownTransform = getWorldTransform();

      auto sunDirection = glm::vec3(mSolarSystem->getSunDirection(ownTransform[3]));

      mModel->setLightDirection(sunDirection.x, sunDirection.y, sunDirection.z);

      if (mSettings->mGraphics.pEnableHDR.get()) {
        mModel->setEnableHDR(true);
        sunIlluminance = static_cast<float>(mSolarSystem->getSunIlluminance(ownTransform[3]));
      }
      mModel->setLightIntensity(sunIlluminance);
    }
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////

} // namespace csp::satellites
