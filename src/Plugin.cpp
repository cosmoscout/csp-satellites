////////////////////////////////////////////////////////////////////////////////////////////////////
//                               This file is part of CosmoScout VR                               //
//      and may be used under the terms of the MIT license. See the LICENSE file for details.     //
//                        Copyright: (c) 2019 German Aerospace Center (DLR)                       //
////////////////////////////////////////////////////////////////////////////////////////////////////

#include "Plugin.hpp"

#include "Satellite.hpp"

#include "../../../src/cs-core/SolarSystem.hpp"
#include "../../../src/cs-utils/convert.hpp"

////////////////////////////////////////////////////////////////////////////////////////////////////

EXPORT_FN cs::core::PluginBase* create() {
  return new csp::satellites::Plugin;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

EXPORT_FN void destroy(cs::core::PluginBase* pluginBase) {
  delete pluginBase;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

namespace csp::satellites {

////////////////////////////////////////////////////////////////////////////////////////////////////

void from_json(const nlohmann::json& j, Plugin::Settings::Transformation& o) {
  nlohmann::json b1 = j.at("translation");
  for (int i = 0; i < 3; ++i) {
    o.mTranslation[i] = b1.at(i);
  }

  nlohmann::json b2 = j.at("rotation");
  for (int i = 0; i < 4; ++i) {
    o.mRotation[i] = b2.at(i);
  }

  o.mScale = j.at("scale").get<double>();
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void from_json(const nlohmann::json& j, Plugin::Settings::Satellite& o) {
  o.mModelFile      = j.at("modelFile").get<std::string>();
  o.mEnvironmentMap = j.at("environmentMap").get<std::string>();
  o.mSize           = j.at("size").get<double>();

  auto iter = j.find("transformation");
  if (iter != j.end()) {
    o.mTransformation = iter->get<std::optional<Plugin::Settings::Transformation>>();
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void from_json(const nlohmann::json& j, Plugin::Settings& o) {
  o.mSatellites = j.at("satellites").get<std::map<std::string, Plugin::Settings::Satellite>>();
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void Plugin::init() {
  std::cout << "Loading: CosmoScout VR Plugin Satellites" << std::endl;

  mPluginSettings = mAllSettings->mPlugins.at("csp-satellites");

  for (auto const& settings : mPluginSettings.mSatellites) {
    auto anchor = mAllSettings->mAnchors.find(settings.first);

    if (anchor == mAllSettings->mAnchors.end()) {
      throw std::runtime_error(
          "There is no Anchor \"" + settings.first + "\" defined in the settings.");
    }

    double tStartExistence = cs::utils::convert::toSpiceTime(anchor->second.mStartExistence);
    double tEndExistence   = cs::utils::convert::toSpiceTime(anchor->second.mEndExistence);

    auto satellite = std::make_shared<Satellite>(settings.second, anchor->second.mCenter,
        anchor->second.mFrame, tStartExistence, tEndExistence, mSceneGraph);

    satellite->setSun(mSolarSystem->getSun());
    mSolarSystem->registerBody(satellite);

    mSatellites.push_back(satellite);
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void Plugin::deInit() {
  for (auto const& satellite : mSatellites) {
    mSolarSystem->unregisterBody(satellite);
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////

} // namespace csp::satellites
