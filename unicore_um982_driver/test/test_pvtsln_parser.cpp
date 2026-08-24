#include <gtest/gtest.h>
#include "unicore_um982_driver/pvtsln_data.hpp"
#include <cmath>

using unicore_um982_driver::PVTSLNData;
using unicore_um982_driver::parsePVTSLN;
using unicore_um982_driver::nedHeadingToEnuYaw;

namespace
{
// Real sample sentences, UM982-CONFIG.txt lines 23-25 (in the submodule root's docs/).
const char* kSample1 =
    "#PVTSLNA,79,GPS,FINE,2377,93246900,0,0,18,8;SINGLE,188.1621,34.60324214605,"
    "-86.74943363150,7.5696,4.3304,3.4834,0.000,SINGLE,188.1621,34.60324214605,"
    "-86.74943363150,-28.5587,19,17,19,17,-0.0933,-0.0469,-0.1506,NONE,0.0000,0.0000,"
    "0.0000,0,0,0,0,2.8451,1.6314,0.8332,2.4753,2.3309,5.0,17,3,4,9,16,26,31,47,49,51,76,"
    "77,82,104,179,182,196,205*d127bb24";
const char* kSample2 =
    "#PVTSLNA,79,GPS,FINE,2377,93246950,0,0,18,8;SINGLE,188.1560,34.60324231724,"
    "-86.74943381326,7.4246,4.2106,3.5052,0.000,SINGLE,188.1560,34.60324231724,"
    "-86.74943381326,-28.5587,19,16,19,16,-0.0893,-0.0504,-0.1533,NONE,0.0000,0.0000,"
    "0.0000,0,0,0,0,2.8745,1.6612,0.8580,2.4978,2.3458,5.0,16,3,4,9,16,31,47,49,51,76,77,"
    "82,104,179,182,196,205*3d95de32";
const char* kSample3 =
    "#PVTSLNA,79,GPS,FINE,2377,93247000,0,0,18,16;SINGLE,188.1447,34.60324236426,"
    "-86.74943393850,7.4653,4.2280,3.5255,0.000,SINGLE,188.1447,34.60324236426,"
    "-86.74943393850,-28.5587,19,16,19,16,-0.0848,-0.0517,-0.1540,NONE,0.0000,0.0000,"
    "0.0000,0,0,0,0,2.8745,1.6612,0.8580,2.4978,2.3458,5.0,16,3,4,9,16,31,47,49,51,76,77,"
    "82,104,179,182,196,205*beb1a7df";
}  // namespace

TEST(PvtslnParser, HuntsvilleSampleFieldMapping)
{
    PVTSLNData data;
    ASSERT_TRUE(parsePVTSLN(kSample1, data));

    EXPECT_NEAR(data.latitude, 34.6032, 1e-3);
    EXPECT_NEAR(data.longitude, -86.7494, 1e-3);
    // idx1 is bestpos height, NOT heading (the original bug's field-off-by-one).
    EXPECT_NEAR(data.altitude_msl, 188.16, 1e-2);
    EXPECT_NEAR(data.undulation, -28.56, 1e-2);
    EXPECT_NEAR(data.sigma_altitude, 7.5696, 1e-4);
    EXPECT_NEAR(data.sigma_latitude, 4.3304, 1e-4);
    EXPECT_NEAR(data.sigma_longitude, 3.4834, 1e-4);
}

TEST(PvtslnParser, SigmasAreNonNegativeOnAllRealSamples)
{
    // This is the assertion that would have caught the original bug: the old field
    // map read velocity fields (which can be negative) into the sigma_* members.
    for (const char* sentence : {kSample1, kSample2, kSample3}) {
        PVTSLNData data;
        ASSERT_TRUE(parsePVTSLN(sentence, data));
        EXPECT_GE(data.sigma_altitude, 0.0);
        EXPECT_GE(data.sigma_latitude, 0.0);
        EXPECT_GE(data.sigma_longitude, 0.0);
    }
}

TEST(PvtslnParser, HeadingTypeIsNoneOnAllRealSamples)
{
    for (const char* sentence : {kSample1, kSample2, kSample3}) {
        PVTSLNData data;
        ASSERT_TRUE(parsePVTSLN(sentence, data));
        EXPECT_EQ(data.heading_type, "NONE");
    }
}

TEST(PvtslnParser, SyntheticHeadingFixParsesAndConvertsNedToEnu)
{
    const char* sentence =
        "#PVTSLNA,79,GPS,FINE,2377,93246900,0,0,18,8;NARROW_INT,188.1621,34.60324214605,"
        "-86.74943363150,7.5696,4.3304,3.4834,0.000,SINGLE,188.1621,34.60324214605,"
        "-86.74943363150,-28.5587,19,17,19,17,-0.0933,-0.0469,-0.1506,NARROW_INT,0.43,"
        "90.0,0.0*ffffffff";

    PVTSLNData data;
    ASSERT_TRUE(parsePVTSLN(sentence, data));

    EXPECT_EQ(data.position_status, "NARROW_INT");
    EXPECT_EQ(data.heading_type, "NARROW_INT");
    EXPECT_NEAR(data.heading_length, 0.43, 1e-9);
    EXPECT_NEAR(data.heading_degree, 90.0, 1e-9);

    // NED 90 degrees (due east) must map to ENU yaw 0.
    double yaw_enu = nedHeadingToEnuYaw(data.heading_degree, 0.0);
    EXPECT_NEAR(yaw_enu, 0.0, 1e-9);
}

TEST(PvtslnParser, MalformedInputReturnsFalseWithoutThrowing)
{
    PVTSLNData data;

    EXPECT_FALSE(parsePVTSLN("", data));
    EXPECT_FALSE(parsePVTSLN("#PVTSLNA,79,GPS,FINE,2377,93246900,0,0,18,8", data));  // no ';'
    EXPECT_FALSE(parsePVTSLN(
        "#PVTSLNA,79,GPS,FINE,2377,93246900,0,0,18,8;SINGLE,188.16,34.6,-86.7,7.5*00000000",
        data));  // only 5 body fields
    EXPECT_FALSE(parsePVTSLN(
        "#PVTSLNA,79,GPS,FINE,2377,93246900,0,0,18,8;SINGLE,abc,34.60324214605,"
        "-86.74943363150,7.5696,4.3304,3.4834,0.000,SINGLE,188.1621,34.60324214605,"
        "-86.74943363150,-28.5587,19,17,19,17,-0.0933,-0.0469,-0.1506,NONE,0.0000,0.0000,"
        "0.0000*00000000",
        data));  // non-numeric field where a double is expected
}
