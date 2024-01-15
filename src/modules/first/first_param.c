/**
 * The angle of arm1 when the aux2<0
 *
 * The angle of arm is -30 degree ,when the parameter is set to -9,
 * The angle of arm is 0 degree ,when the parameter is set to 5,
 * The angle of arm is 90 degree ,when the parameter is set to 9
 *
 * @min -10
 * @max 10
 * @decimal 1
 * @increment 5
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_INT32(ANGLE_ONE, -5);

/**
 * The angle of arm2 when the aux2<0
 *
 * The angle of arm is -30 degree ,when the parameter is set to -9,
 * The angle of arm is 0 degree ,when the parameter is set to 5,
 * The angle of arm is 90 degree ,when the parameter is set to 9
 *
 * @min -10
 * @max 10
 * @decimal 1
 * @increment 5
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_INT32(ANGLE_TWO, -5);

/**
 * The angle of arm3 when the aux2<0
 *
 * The angle of arm is -30 degree ,when the parameter is set to -9,
 * The angle of arm is 0 degree ,when the parameter is set to 5,
 * The angle of arm is 90 degree ,when the parameter is set to 9
 *
 * @min -10
 * @max 10
 * @decimal 1
 * @increment 5
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_INT32(ANGLE_THREE, -5);

/**
 * The angle of arm4 when the aux2<0
 *
 * The angle of arm is -30 degree ,when the parameter is set to -9,
 * The angle of arm is 0 degree ,when the parameter is set to 5,
 * The angle of arm is 90 degree ,when the parameter is set to 9
 *
 * @min -10
 * @max 10
 * @decimal 1
 * @increment 5
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_INT32(ANGLE_FOUR, -5);

/**
 * distance to do mm
 *
 * @min 1500
 * @max 2000
 * @decimal 1
 * @increment 5
 * @group Multicopter Attitude Control
*/
PARAM_DEFINE_INT32(DISTANCE, 1600);
