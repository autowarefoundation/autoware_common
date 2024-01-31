// TODO: Add license header
// TODO: remove unused includes
#include "lanelet2_extension/autoware_lanelet2_validation/unconnected_relations.hpp"

#include <gtest/gtest.h>

using lanelet::AttributeMap;
using lanelet::AttributeName;
using lanelet::AttributeValueString;
using lanelet::Lanelet;
using lanelet::LaneletMapPtr;
using lanelet::LineString3d;
using lanelet::Point3d;
using lanelet::Polygon3d;
using lanelet::RegulatoryElementPtr;
using lanelet::TrafficLight;
using lanelet::autoware::Crosswalk;
using lanelet::utils::getId;
class TestSuite : public ::testing::Test
{
public:
  TestSuite()
  {
    // TODO: remove
    lanelet::validation::RegisterMapValidator<lanelet::validation::UnconnectedRelationsChecker>();

    initializeAttributes();
    config.checksFilter = "mapping.unconnected_relations";
    validators = lanelet::validation::availabeChecks(config.checksFilter);
  }

  ~TestSuite() override = default;

  void initializeAttributes()
  {
    // Stop Line
    sl_attr[AttributeName::Type] = AttributeValueString::StopLine;

    // Traffic Light
    tl_attr[AttributeName::Type] = AttributeValueString::TrafficLight;
    tl_attr[AttributeName::Subtype] = "red_yellow_green";
    tl_attr["height"] = "0.5";

    // Crosswalk polygon
    cw_poly_attr[AttributeName::Type] =
      lanelet::autoware::Crosswalk::AutowareRoleNameString::CrosswalkPolygon;

    // Crosswalk
    cw_attr[AttributeName::Type] = AttributeValueString::Lanelet;
    cw_attr[AttributeName::Subtype] = AttributeValueString::Crosswalk;
    cw_attr[AttributeName::SpeedLimit] = "10";
    cw_attr[AttributeName::OneWay] = "no";
    cw_attr[AttributeName::Location] = AttributeValueString::Urban;
    cw_attr[AttributeName::ParticipantPedestrian] = "yes";

    // Regulatory element of traffic light
    AttributeMap tl_re_attr = AttributeMap();
    tl_re_attr[AttributeName::Type] = AttributeValueString::RegulatoryElement;
    tl_re_attr[AttributeName::Subtype] = AttributeValueString::TrafficLight;

    // Regulatory element of crosswalk
    AttributeMap cw_re_attr = AttributeMap();
    cw_re_attr[AttributeName::Type] = AttributeValueString::RegulatoryElement;
    cw_re_attr[AttributeName::Subtype] = AttributeValueString::Crosswalk;
  }

  void addTestMap(LaneletMapPtr in_map_ptr)
  {
    // Create test map

    // Points for stop line
    Point3d p0 = Point3d(getId(), 0.0, 0.0, 0.1);
    Point3d p1 = Point3d(getId(), 0.0, 1.0, 0.1);
    Point3d p2 = Point3d(getId(), 0.0, 2.0, 0.1);
    // Points for traffic light
    Point3d p3 = Point3d(getId(), 0.0, 0.0, 5.0);
    Point3d p4 = Point3d(getId(), 0.0, 1.0, 5.0);
    Point3d p5 = Point3d(getId(), 0.0, 2.0, 5.0);
    // Points for crosswalk
    Point3d p6 = Point3d(getId(), 1.0, 0.0, 0.1);
    Point3d p7 = Point3d(getId(), 1.0, 1.0, 0.1);
    Point3d p8 = Point3d(getId(), 1.0, 2.0, 0.1);
    Point3d p9 = Point3d(getId(), 2.0, 0.0, 0.1);
    Point3d p10 = Point3d(getId(), 2.0, 1.0, 0.1);
    Point3d p11 = Point3d(getId(), 2.0, 2.0, 0.1);

    // Stop line
    LineString3d sl1 = LineString3d(getId(), {p0, p1}, sl_attr);
    LineString3d sl2 = LineString3d(getId(), {p1, p2}, sl_attr);

    LineString3d tl1 = LineString3d(getId(), {p3, p4}, tl_attr);
    LineString3d tl2 = LineString3d(getId(), {p4, p5}, tl_attr);

    // LineStrings for crosswalk
    LineString3d cw_ls1 = LineString3d(getId(), {p6, p7});
    LineString3d cw_ls2 = LineString3d(getId(), {p7, p8});
    LineString3d cw_ls3 = LineString3d(getId(), {p9, p10});
    LineString3d cw_ls4 = LineString3d(getId(), {p10, p11});

    Polygon3d cw_poly1 = Polygon3d(getId(), {p7, p6, p9, p10, p7}, cw_poly_attr);
    Polygon3d cw_poly2 = Polygon3d(getId(), {p8, p7, p10, p11, p8}, cw_poly_attr);
    // Lanelets for crosswalk
    Lanelet cw1 = Lanelet(getId(), cw_ls1, cw_ls3, cw_attr);
    Lanelet cw2 = Lanelet(getId(), cw_ls2, cw_ls4, cw_attr);

    // Traffic light regulatory element
    RegulatoryElementPtr tl_reg_elem1, tl_reg_elem2;
    tl_reg_elem1 = TrafficLight::make(getId(), tl_re_attr, {tl1}, {sl1});
    tl_reg_elem2 = TrafficLight::make(getId(), tl_re_attr, {tl2}, {sl2});

    // Crosswalk regulatory element
    RegulatoryElementPtr cw_reg_elem1, cw_reg_elem2;
    cw_reg_elem1 = Crosswalk::make(getId(), cw_re_attr, cw1, cw_poly1, {sl1});
    cw_reg_elem2 = Crosswalk::make(getId(), cw_re_attr, cw2, cw_poly2, {sl2});

    cw1.addRegulatoryElement(cw_reg_elem1);
    cw2.addRegulatoryElement(cw_reg_elem2);

    // Add elements to map
    for (const auto & re : {tl_reg_elem1, tl_reg_elem2}) {
      in_map_ptr->add(re);
    }
    for (const auto & cw : {cw1, cw2}) {
      in_map_ptr->add(cw);
    }
  }
  AttributeMap sl_attr, tl_attr, cw_attr, cw_poly_attr, tl_re_attr, cw_re_attr;
  lanelet::validation::ValidationConfig config;
  lanelet::validation::Strings validators;

private:
};

// TEST_F(TestSuite, SaveMap)  // NOLINT for gtest
// {
//   // Save map
//   lanelet::write("/tmp/test_map.osm", *test_map_ptr);
// }

TEST_F(TestSuite, AvailableValidator)  // NOLINT for gtest
{
  unsigned int expected_num_validators = 1;
  EXPECT_EQ(expected_num_validators, validators.size());
  for (const auto & v : validators) {
    EXPECT_EQ("mapping.unconnected_relations", v);
  }
}

TEST_F(TestSuite, MissingRegulatoryElementOfTrafficLight)  // NOLINT for gtest
{
  // Check missing regulatory element of traffic light

  const auto & tl_no_reg_elem = LineString3d(
    99999, {Point3d(getId(), 0.0, 3.0, 5.0), Point3d(getId(), 0.0, 4.0, 5.0)}, tl_attr);
  LaneletMapPtr test_map_ptr = lanelet::utils::createMap({tl_no_reg_elem});
  addTestMap(test_map_ptr);

  auto detect_issues = lanelet::validation::validateMap(*test_map_ptr, config);
  for (const auto & detect_issue : detect_issues) {
    for (const auto & issue : detect_issue.issues) {
      std::cout << "message = " << issue.message << std::endl;
      EXPECT_EQ(99999, issue.id);
      EXPECT_EQ(lanelet::validation::Severity::Error, issue.severity);
      EXPECT_EQ(lanelet::validation::Primitive::LineString, issue.primitive);
    }
  }
}

TEST_F(TestSuite, MissingRegulatoryElementOfCrosswalk)  // NOLINT for gtest
{
  // Check missing regulatory element of crosswalk

  const auto & cw_no_reg_elem = Lanelet(
    99999,
    LineString3d(getId(), {Point3d(getId(), 3.0, 0.0, 0.1), Point3d(getId(), 3.0, 1.0, 0.1)}),
    LineString3d(getId(), {Point3d(getId(), 3.0, 1.0, 0.1), Point3d(getId(), 3.0, 2.0, 0.1)}),
    cw_attr);
  LaneletMapPtr test_map_ptr = lanelet::utils::createMap({cw_no_reg_elem});
  addTestMap(test_map_ptr);

  auto detect_issues = lanelet::validation::validateMap(*test_map_ptr, config);
  for (const auto & detect_issue : detect_issues) {
    for (const auto & issue : detect_issue.issues) {
      std::cout << "message = " << issue.message << std::endl;
      EXPECT_EQ(99999, issue.id);
      // EXPECT_EQ(lanelet::validation::Severity::Error, issue.severity);
      // EXPECT_EQ(lanelet::validation::Primitive::Lanelet, issue.primitive);
    }
  }
}

TEST_F(TestSuite, RegulatoryElementofTrafficLightWithoutTrafficLight)  // NOLINT for gtest
{
  // Check regulatory element of traffic light without stop line

  RegulatoryElementPtr tl_reg_elem_no_tl;
  const auto & tl = LineString3d(99998, {});
  LaneletMapPtr test_map_ptr = lanelet::utils::createMap({tl});
  tl_reg_elem_no_tl = TrafficLight::make(
    99999, tl_re_attr, {tl},
    {LineString3d(
      getId(), {Point3d(getId(), 3.0, 3.0, 0.1), Point3d(getId(), 3.0, 4.0, 0.1)}, sl_attr)});
  test_map_ptr->add(tl_reg_elem_no_tl);
  addTestMap(test_map_ptr);

  auto detect_issues = lanelet::validation::validateMap(*test_map_ptr, config);
  for (const auto & detect_issue : detect_issues) {
    for (const auto & issue : detect_issue.issues) {
      std::cout << "message = " << issue.message << std::endl;
      EXPECT_TRUE(issue.id == 99999 || issue.id == 99998);
      // EXPECT_EQ(lanelet::validation::Severity::Error, issue.severity);
      // EXPECT_EQ(lanelet::validation::Primitive::LineString, issue.primitive);
    }
  }
}

TEST_F(TestSuite, RegulatoryElementofTrafficLightWithoutStopLine)  // NOLINT for gtest
{
  // Check regulatory element of traffic light without stop line

  RegulatoryElementPtr tl_reg_elem_no_sl;
  const auto & tl = LineString3d(
    getId(), {Point3d(getId(), 0.0, 3.0, 5.0), Point3d(getId(), 0.0, 4.0, 5.0)}, tl_attr);
  LaneletMapPtr test_map_ptr = lanelet::utils::createMap({tl});
  tl_reg_elem_no_sl = TrafficLight::make(99999, tl_re_attr, {tl}, {});
  test_map_ptr->add(tl_reg_elem_no_sl);
  addTestMap(test_map_ptr);

  auto detect_issues = lanelet::validation::validateMap(*test_map_ptr, config);
  for (const auto & detect_issue : detect_issues) {
    for (const auto & issue : detect_issue.issues) {
      std::cout << "message = " << issue.message << std::endl;
      EXPECT_EQ(99999, issue.id);
      // EXPECT_EQ(lanelet::validation::Severity::Error, issue.severity);
      // EXPECT_EQ(lanelet::validation::Primitive::LineString, issue.primitive);
    }
  }
}

TEST_F(TestSuite, RegulatoryElementOfCrosswalkWithoutPolygon)  // NOLINT for gtest
{
  // Check regulatory element of crosswalk without polygon

  RegulatoryElementPtr cw_reg_elem_no_poly;
  Lanelet cw_no_poly = Lanelet(
    getId(),
    LineString3d(getId(), {Point3d(getId(), 3.0, 0.0, 0.1), Point3d(getId(), 3.0, 1.0, 0.1)}),
    LineString3d(getId(), {Point3d(getId(), 3.0, 1.0, 0.1), Point3d(getId(), 3.0, 2.0, 0.1)}),
    cw_attr);

  RegulatoryElementPtr reg_elem = Crosswalk::make(
    99999, cw_re_attr, cw_no_poly, Polygon3d(99998),
    {LineString3d(
      getId(), {Point3d(getId(), 3.0, 3.0, 0.1), Point3d(getId(), 3.0, 4.0, 0.1)}, sl_attr)});
  cw_no_poly.addRegulatoryElement(reg_elem);
  LaneletMapPtr test_map_ptr = lanelet::utils::createMap({cw_no_poly});
  addTestMap(test_map_ptr);

  auto detect_issues = lanelet::validation::validateMap(*test_map_ptr, config);
  for (const auto & detect_issue : detect_issues) {
    for (const auto & issue : detect_issue.issues) {
      std::cout << "message = " << issue.message << std::endl;
      EXPECT_TRUE(issue.id == 99999 || issue.id == 99998);
      // EXPECT_EQ(lanelet::validation::Severity::Error, issue.severity);
      // EXPECT_EQ(lanelet::validation::Primitive::LineString, issue.primitive);
    }
  }
}

TEST_F(TestSuite, RegulatoryElementOfCrosswalkWithoutStopline)  // NOLINT for gtest
{
  // Check regulatory element of crosswalk without stop line

  RegulatoryElementPtr cw_reg_elem_no_sl;
  Lanelet cw_no_sl = Lanelet(
    getId(),
    LineString3d(getId(), {Point3d(getId(), 3.0, 0.0, 0.1), Point3d(getId(), 3.0, 1.0, 0.1)}),
    LineString3d(getId(), {Point3d(getId(), 3.0, 1.0, 0.1), Point3d(getId(), 3.0, 2.0, 0.1)}),
    cw_attr);

  RegulatoryElementPtr reg_elem = Crosswalk::make(
    99999, cw_re_attr, cw_no_sl,
    Polygon3d(getId(), {Point3d(getId(), 3.0, 3.0, 0.1)}, cw_poly_attr), {});
  cw_no_sl.addRegulatoryElement(reg_elem);
  LaneletMapPtr test_map_ptr = lanelet::utils::createMap({cw_no_sl});
  addTestMap(test_map_ptr);

  auto detect_issues = lanelet::validation::validateMap(*test_map_ptr, config);
  for (const auto & detect_issue : detect_issues) {
    for (const auto & issue : detect_issue.issues) {
      std::cout << "message = " << issue.message << std::endl;
      EXPECT_EQ(99999, issue.id);
      // EXPECT_EQ(lanelet::validation::Severity::Error, issue.severity);
      // EXPECT_EQ(lanelet::validation::Primitive::LineString, issue.primitive);
    }
  }
}

TEST_F(TestSuite, RegulatoryElementOfCrosswalkWithoutCrosswalk)  // NOLINT for gtest
{
  // Check regulatory element of crosswalk without crosswalk

  const auto poly_no_cw = Polygon3d(getId(), {Point3d(getId(), 3.0, 3.0, 0.1)}, cw_poly_attr);
  const auto ll = Lanelet(
    99998,
    LineString3d(getId(), {Point3d(getId(), 3.0, 0.0, 0.1), Point3d(getId(), 3.0, 1.0, 0.1)}),
    LineString3d(getId(), {Point3d(getId(), 3.0, 1.0, 0.1), Point3d(getId(), 3.0, 2.0, 0.1)}));

  RegulatoryElementPtr reg_elem = Crosswalk::make(
    99999, cw_re_attr, ll, poly_no_cw,
    {LineString3d(
      getId(), {Point3d(getId(), 3.0, 3.0, 0.1), Point3d(getId(), 3.0, 4.0, 0.1)}, sl_attr)});
  LaneletMapPtr test_map_ptr = lanelet::utils::createMap({poly_no_cw});
  addTestMap(test_map_ptr);
  test_map_ptr->add(reg_elem);

  // TODO: remove
  // lanelet::write("/tmp/test_map.osm", *test_map_ptr);

  auto detect_issues = lanelet::validation::validateMap(*test_map_ptr, config);
  for (const auto & detect_issue : detect_issues) {
    for (const auto & issue : detect_issue.issues) {
      std::cout << "message = " << issue.message << std::endl;
      EXPECT_TRUE(issue.id == 99999 || issue.id == 99998);
      // EXPECT_EQ(lanelet::validation::Severity::Error, issue.severity);
      // EXPECT_EQ(lanelet::validation::Primitive::LineString, issue.primitive);
    }
  }
}
