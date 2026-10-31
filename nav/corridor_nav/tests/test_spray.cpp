#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "corridor_nav/spray.hpp"

using namespace corridor_nav;

TEST(Spray, DoseTimeAreaAndFixed) {
  Config cfg;
  EXPECT_DOUBLE_EQ(doseTime(4.0, cfg), 2.0);    // 1.2 s -> dose_min_s
  EXPECT_NEAR(doseTime(12.0, cfg), 3.6, 1e-12);
  EXPECT_DOUBLE_EQ(doseTime(40.0, cfg), 6.0);   // 12 s -> dose_max_s
  // A broken outline never yields a zero or NaN dose.
  EXPECT_DOUBLE_EQ(doseTime(std::numeric_limits<double>::quiet_NaN(), cfg), cfg.dose_min_s);
  EXPECT_DOUBLE_EQ(doseTime(std::numeric_limits<double>::infinity(), cfg), cfg.dose_min_s);
  EXPECT_DOUBLE_EQ(doseTime(-3.0, cfg), cfg.dose_min_s);
  cfg.dose_mode = DoseMode::kFixed;
  EXPECT_DOUBLE_EQ(doseTime(12.0, cfg), 4.0);
}

TEST(Spray, DoseValidOnlyAboveFraction) {
  // Open from t0 to t0+3.4 (85 % of 4 s), then closed.
  DoseTracker a(0.8);
  a.start(10.0, 4.0);
  a.update(10.0, true);
  a.update(11.0, true);
  a.update(13.4, true);
  a.update(13.5, false);       // interval 13.4..13.5 was open -> 3.5 s open in total
  EXPECT_FALSE(a.done(13.9));
  a.update(14.0, false);
  EXPECT_TRUE(a.done(14.0));
  EXPECT_NEAR(a.openFraction(), 3.5 / 4.0, 1e-9);
  EXPECT_TRUE(a.valid());

  DoseTracker b(0.8);
  b.start(0.0, 4.0);
  b.update(0.0, true);
  b.update(3.0, false);        // open 3.0 s = 75 %
  b.update(4.0, false);
  EXPECT_TRUE(b.done(4.0));
  EXPECT_NEAR(b.openFraction(), 0.75, 1e-9);
  EXPECT_FALSE(b.valid());

  DoseTracker c(0.8);
  c.start(0.0, 4.0);
  c.update(0.0, true);
  c.update(3.4, false);        // exactly 85 %
  c.update(4.0, false);
  EXPECT_NEAR(c.openFraction(), 0.85, 1e-9);
  EXPECT_TRUE(c.valid());
}

TEST(Spray, DoseUsesPreviousFlagAndIgnoresTimePastTheDose) {
  DoseTracker d(0.8);
  d.start(0.0, 2.0);
  d.update(1.0, false);        // closed during 0..1
  d.update(5.0, true);         // closed during 1..5 (previous flag), open flag stored
  EXPECT_DOUBLE_EQ(d.openFraction(), 0.0);
  d.update(9.0, true);         // open 5..9 but all past t0 + dose
  EXPECT_DOUBLE_EQ(d.openFraction(), 0.0);

  DoseTracker e(0.8);
  e.start(0.0, 2.0);
  e.update(0.0, true);
  e.update(1.5, true);
  e.update(4.0, false);        // open 1.5..4.0, only 1.5..2.0 counts -> 2.0 s of 2.0 s
  EXPECT_DOUBLE_EQ(e.openFraction(), 1.0);
  EXPECT_TRUE(e.valid());
}

TEST(Spray, DoseStartResetsAndZeroDoseIsInvalid) {
  DoseTracker d(0.8);
  d.start(0.0, 2.0);
  d.update(0.0, true);
  d.update(2.0, true);
  EXPECT_TRUE(d.valid());
  d.start(10.0, 2.0);
  EXPECT_DOUBLE_EQ(d.openFraction(), 0.0);
  EXPECT_FALSE(d.valid());
  EXPECT_FALSE(d.done(10.0));
  EXPECT_TRUE(d.done(12.0));
  d.start(0.0, 0.0);
  EXPECT_DOUBLE_EQ(d.openFraction(), 0.0);
}
