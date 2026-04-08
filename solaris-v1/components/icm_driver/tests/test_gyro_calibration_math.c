/*
 * test_gyro_calibration_math.c
 *
 * BDD suite — Gyroscope Scale-Factor Calculation
 *
 * Feature: Each ICM20948 chip has a slightly different PLL oscillator
 *          frequency due to manufacturing variance.  The chip stores a
 *          signed 8-bit trim value in the TIMEBASE_CORRECTION_PLL register.
 *          calcGyroSf() converts this trim into a per-chip scale factor
 *          written to the DMP so that gyroscope readings in °/s are accurate.
 *
 *          The relationship is:
 *            realFreq = 102870 + 81 × pllTrim
 *            gyroSf   ∝ 1 / realFreq
 *
 *          The result is a 32-bit integer; the DMP interprets it in its own
 *          internal fixed-point format.
 *
 *          This is pure arithmetic — no SPI access occurs.
 *
 * Function under test: ICM20948_calcGyroSf  (static; accessible because
 *                       icm20948.c is #included directly)
 */

#include "mock_common.h"
#include "icm20948.c"

Describe(CalcGyroSf);
BeforeEach(CalcGyroSf) { mock_reset(); }
AfterEach(CalcGyroSf)  {}

/*
 * GIVEN pll trim = 0 (ideal chip, no correction needed)
 * WHEN  calcGyroSf is called
 * THEN  it returns the expected nominal scale factor 83290354
 *       (validated against the reference implementation)
 */
Ensure(CalcGyroSf, it_returns_the_expected_nominal_value_for_zero_pll_trim)
{
    spp_int32_t sf = ICM20948_calcGyroSf(0);

    assert_that(sf, is_equal_to(83290354L));
}

/*
 * GIVEN a positive pll trim (oscillator runs faster than nominal)
 * WHEN  calcGyroSf is called
 * THEN  the scale factor is smaller than the nominal value
 *       (higher frequency → smaller reciprocal)
 */
Ensure(CalcGyroSf, it_returns_a_smaller_value_for_a_higher_pll_frequency)
{
    spp_int32_t sfNominal  = ICM20948_calcGyroSf(0);
    spp_int32_t sfPositive = ICM20948_calcGyroSf(10);

    assert_that(sfPositive, is_less_than(sfNominal));
}

/*
 * GIVEN a negative pll trim (oscillator runs slower than nominal)
 * WHEN  calcGyroSf is called
 * THEN  the scale factor is larger than the nominal value
 *       (lower frequency → larger reciprocal)
 */
Ensure(CalcGyroSf, it_returns_a_larger_value_for_a_lower_pll_frequency)
{
    spp_int32_t sfNominal  = ICM20948_calcGyroSf(0);
    spp_int32_t sfNegative = ICM20948_calcGyroSf(-10);

    assert_that(sfNegative, is_greater_than(sfNominal));
}

/*
 * GIVEN pll trim = +127 (maximum positive value)
 * WHEN  calcGyroSf is called
 * THEN  the result is positive (no integer overflow)
 */
Ensure(CalcGyroSf, it_returns_a_positive_result_for_the_maximum_positive_trim)
{
    spp_int32_t sf = ICM20948_calcGyroSf(127);

    assert_that(sf, is_greater_than(0L));
}

/*
 * GIVEN pll trim = -128 (maximum negative value)
 * WHEN  calcGyroSf is called
 * THEN  the result is positive and larger than the +127 result
 *       (lower frequency → larger scale; no overflow)
 */
Ensure(CalcGyroSf, it_returns_a_positive_result_for_the_maximum_negative_trim)
{
    spp_int32_t sfMax = ICM20948_calcGyroSf(127);
    spp_int32_t sfMin = ICM20948_calcGyroSf(-128);

    assert_that(sfMin, is_greater_than(0L));
    assert_that(sfMin, is_greater_than(sfMax));
}

/* ================================================================
 * Test runner
 * ================================================================ */

int main(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    TestSuite *suite = create_test_suite();

    add_test_with_context(suite, CalcGyroSf, it_returns_the_expected_nominal_value_for_zero_pll_trim);
    add_test_with_context(suite, CalcGyroSf, it_returns_a_smaller_value_for_a_higher_pll_frequency);
    add_test_with_context(suite, CalcGyroSf, it_returns_a_larger_value_for_a_lower_pll_frequency);
    add_test_with_context(suite, CalcGyroSf, it_returns_a_positive_result_for_the_maximum_positive_trim);
    add_test_with_context(suite, CalcGyroSf, it_returns_a_positive_result_for_the_maximum_negative_trim);

    return run_test_suite(suite, create_text_reporter());
}
