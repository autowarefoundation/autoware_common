// Copyright 2023 Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef LANELET2_EXTENSION__IO__AUTOWARE_MULTI_OSM_PARSER_HPP_
#define LANELET2_EXTENSION__IO__AUTOWARE_MULTI_OSM_PARSER_HPP_

#include "lanelet2_io/Exceptions.h"
#include "lanelet2_io/io_handlers/Factory.h"
#include "lanelet2_io/io_handlers/OsmFile.h"
#include "lanelet2_io/io_handlers/OsmHandler.h"

#include <boost/geometry/algorithms/is_valid.hpp>

#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_core/geometry/Polygon.h>

#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

using std::string_literals::operator""s;

namespace lanelet
{
namespace io_handlers
{

class MultiOsmParser : public Parser
{
public:
  using Parser::Parser;

  std::unique_ptr<LaneletMap> parse(
    const std::string & lanelet2_filename, ErrorMessages & errors) const override;

  std::unique_ptr<LaneletMap> parse(
    const std::vector<std::string> & lanelet2_filenames, ErrorMessages & errors) const;

  std::unique_ptr<LaneletMap> fromOsmFile(const osm::File & file, ErrorMessages & errors) const;

  std::unique_ptr<LaneletMap> fromOsmFile(
    const std::map<std::string, osm::File> & files_map, ErrorMessages & errors) const;

  static void parseVersions(
    const std::string & filename, std::string * format_version, std::string * map_version);

  static constexpr const char * extension() { return ".osm"; }

  static constexpr const char * name() { return "autoware_multi_osm_handler"; }
};

namespace
{
RegisterParser<MultiOsmParser> regParser;

class MultiFileLoader
{
public:
  static std::unique_ptr<LaneletMap> loadMap(
    const osm::File & file, const Projector & projector, ErrorMessages & errors)
  {
    MultiFileLoader loader;
    loader.loadNodes(file.nodes, projector);
    loader.loadWays(file.ways);
    auto laneletsWithRelation = loader.loadLanelets(file.relations);
    auto areasWithRelation = loader.loadAreas(file.relations);
    loader.loadRegulatoryElements(file.relations);
    loader.addRegulatoryElements(laneletsWithRelation);
    loader.addRegulatoryElements(areasWithRelation);
    errors = std::move(loader.errors_);

    return std::make_unique<LaneletMap>(
      loader.lanelets_, loader.areas_, loader.regulatoryElements_, loader.polygons_,
      loader.lineStrings_, loader.points_);
  }

  static std::unique_ptr<LaneletMap> loadMap(
    const std::map<std::string, osm::File> & files_map, const Projector & projector,
    ErrorMessages & errors)
  {
    MultiFileLoader loader;

    std::for_each(files_map.begin(), files_map.end(), [&loader, &projector](const auto & file) {
      loader.loadNodes(file.second.nodes, projector);
    });

    std::for_each(files_map.begin(), files_map.end(), [&loader](const auto & file) {
      loader.loadWays(file.second.ways);
    });

    LaneletsWithRegulatoryElements laneletsWithRelation;
    for (const auto & file : files_map) {
      laneletsWithRelation = loader.loadLanelets(file.second.relations);
    }

    AreasWithRegulatoryElements areasWithRelation;
    for (const auto & file : files_map) {
      areasWithRelation = loader.loadAreas(file.second.relations);
    }

    for (const auto & file : files_map) {
      loader.loadRegulatoryElements(file.second.relations);
    }
    loader.addRegulatoryElements(laneletsWithRelation);
    loader.addRegulatoryElements(areasWithRelation);

    errors = std::move(loader.errors_);
    return std::make_unique<LaneletMap>(
      loader.lanelets_, loader.areas_, loader.regulatoryElements_, loader.polygons_,
      loader.lineStrings_, loader.points_);
  }

private:
  std::vector<std::string> errors_;
  LaneletLayer::Map lanelets_;
  AreaLayer::Map areas_;
  RegulatoryElementLayer::Map regulatoryElements_;
  PolygonLayer::Map polygons_;
  LineStringLayer::Map lineStrings_;
  PointLayer::Map points_;

public:
  template <typename PrimT>
  using PrimitiveWithRegulatoryElement = std::pair<PrimT, const osm::Relation *>;

  template <typename PrimT>
  using PrimitivesWithRegulatoryElement = std::vector<PrimitiveWithRegulatoryElement<PrimT>>;

  using AreasWithRegulatoryElements = PrimitivesWithRegulatoryElement<Area>;
  using LaneletsWithRegulatoryElements = PrimitivesWithRegulatoryElement<Lanelet>;

  MultiFileLoader() = default;

  void loadNodes(const lanelet::osm::Nodes & nodes, const Projector & projector);

  void loadWays(const lanelet::osm::Ways & ways);

  LaneletsWithRegulatoryElements loadLanelets(const lanelet::osm::Relations & relations);

  AreasWithRegulatoryElements loadAreas(const lanelet::osm::Relations & relations);

  void loadRegulatoryElements(const osm::Relations & relations);

  template <typename PrimT>
  void addRegulatoryElements(std::vector<std::pair<PrimT, const osm::Relation *>> & addTos);

  template <const char * Type>
  bool isType(const lanelet::osm::Relation & relation);

  static lanelet::AttributeMap getAttributes(const lanelet::osm::Attributes & osmAttributes);

  LineString3d getLaneletBorder(const osm::Relation & llElem, const std::string & role);

  LineStrings3d getLinestrings(const osm::Roles & roles, const std::string & roleName, Id refId);

  LineStrings3d getOuterRing(const osm::Relation & area);

  std::vector<LineStrings3d> getInnerRing(const osm::Relation & area);

  RuleParameterMap getRulesForRegulatoryElement(Id currElemId, const osm::Roles & roles);

  std::vector<LineStrings3d> assembleBoundary(LineStrings3d lineStrings, Id id);

  template <typename PrimT>
  PrimT getOrGetDummy(
    const typename std::unordered_map<Id, PrimT> & map, Id id, Id currentPrimitiveId);

  void parserError(Id id, const std::string & what);
};
}  // namespace
}  // namespace io_handlers
};  // namespace lanelet

#endif  // LANELET2_EXTENSION__IO__AUTOWARE_MULTI_OSM_PARSER_HPP_
