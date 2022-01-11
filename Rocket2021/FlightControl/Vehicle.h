#define FLIGHT_NUMBER   0       // either 0 or 1

#define DRY_MASS        6.160   // kilograms

#define ATM_DENSITY     1.225   // kg/m^3

#define REF_AREA_BODY   0.0081  // m^2

#define REF_AREA_FLAP   0.0016  // m^2

#define CD_PASSIVE      0.5     // unitless

#define CD_ACTIVE       0.6     // unitless

#define TARGET_ALT      1050    // meters (3000 feet = 914 meters)

#define K_PASSIVE       (0.5 * ATM_DENSITY * CD_PASSIVE * REF_AREA_BODY)

#define K_ACTIVE        (0.5 * ATM_DENSITY * CD_ACTIVE * (REF_AREA_BODY + 4*REF_AREA_FLAP))
