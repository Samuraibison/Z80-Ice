/**
 * @brief  To reduce the number of device types, the low voltage PICs (LF types)
 *         are treated as high voltage types by defining the high voltage name
 *
 * Copyright Debug Innovations Ltd. 2018-2022
 */


#if defined(_18LF25K83) && !defined(_18F25K83)
#define _18F25K83
#endif

#if defined(_18LF26K83) && !defined(_18F26K83)
#define _18F26K83
#endif

#if defined(_18LF26Q10) && !defined(_18F26Q10)
#define _18F26Q10
#endif

#if defined(_18LF26Q43) && !defined(_18F26Q43)
#define _18F26Q43
#endif

#if defined(_18LF27Q43) && !defined(_18F27Q43)
#define _18F27Q43
#endif

